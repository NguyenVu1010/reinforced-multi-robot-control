#!/usr/bin/env python3

# trainer_logic/training_orchestrator.py

import rospy
import torch
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point
import random
import threading
import queue
import time
from collections import namedtuple

from tf.transformations import quaternion_from_euler
from env.environment import MiRPathFollowingEnv 
from core.memory import Memory
from core.trainer import PPOTrainer
from mir_control.srv import RequestPath, RequestPathRequest,ProvidePath, ProvidePathRequest
from utils.training_logger import TrainingLogger

# --- Cấu trúc dữ liệu ---
Experience = namedtuple('Experience', ['state', 'action', 'reward', 'done', 'log_prob'])
WorkerPayload = namedtuple('WorkerPayload', ['robot_name', 'experience', 'goal_reached', 'crashed'])


class TrainingOrchestrator:
    """
    Điều phối vòng lặp huấn luyện theo kiến trúc Actor-Learner đa luồng.
    """
    def __init__(self, config_manager, model_manager, system_controller):
        self.cfg_manager = config_manager
        self.model_manager = model_manager
        self.sys_controller = system_controller
        
        self.num_robots = self.cfg_manager.env_cfg['num_robots']
        self.robot_names = [f'robot{i+1}' for i in range(self.num_robots)]

        # --- Thành phần cho đa luồng ---
        self.experience_queue = queue.Queue(maxsize=self.num_robots * 20)
        self.stop_event = threading.Event()
        self.worker_threads = []

        # --- Khởi tạo môi trường ---
        rospy.loginfo(f"Khởi tạo {self.num_robots} môi trường Gym (phi tập trung) cho: {self.robot_names}")
        self.envs = {
            name: MiRPathFollowingEnv(name, self.robot_names, self.cfg_manager) 
            for name in self.robot_names
        }
        
        # --- Các thành phần của Learner ---
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        ppo_trainer_params = self.cfg_manager.train_cfg.copy()
        ppo_trainer_params.pop('epochs', None)
        ppo_trainer_params.pop('log_interval', None)
        ppo_trainer_params.pop('n_steps', None) 
        
        self.logger = TrainingLogger(self.model_manager.save_dir, self.robot_names, self.cfg_manager.checkpoint_cfg['log_filename'])
        self.trainer = PPOTrainer(
            policy_net=self.model_manager.policy_net, 
            value_net=self.model_manager.value_net, 
            **ppo_trainer_params
        )
        self.memory = Memory()
        self.start_cycle = 0

        # Kết nối service
        self._connect_ros_infra()
        
    def _connect_ros_infra(self):
        rospy.loginfo("Đang chờ service 'request_path_service'...")
        self.service_name = 'request_path_service'
        try:
            rospy.wait_for_service(self.service_name, timeout=30.0)
            self.RequestPath_client = rospy.ServiceProxy(self.service_name,RequestPath)
        except rospy.ROSException as e:
            raise RuntimeError(f"Không thể kết nối service 'request_path_service': {e}")

    def run(self):
        """Hàm chạy vòng lặp huấn luyện chính, đóng vai trò là Learner."""
        self.sys_controller.cleanup_control_files()
        self.start_cycle = self.model_manager.load_checkpoints()

        # --- Giai đoạn thiết lập ban đầu ---
        rospy.loginfo("Thiết lập ban đầu và reset tất cả robot...")
        self.reset_all_models()
        for name in self.robot_names:
            if hasattr(self.envs[name], '_wait_for_initial_data'):
                self.envs[name]._wait_for_initial_data()
            else:
                rospy.logwarn(f"Lớp Env cho {name} không có phương thức '_wait_for_initial_data'. Tiếp tục mà không chờ.")
            if not self.request_and_set_new_path(name):
                rospy.logfatal(f"Không thể lấy path ban đầu cho {name}. Dừng chương trình.")
                return
            self.envs[name].reset()
        rospy.loginfo("Thiết lập ban đầu hoàn tất.")

        # --- Khởi chạy các luồng Actor ---
        rospy.loginfo("Bắt đầu khởi chạy các luồng Actor (worker)...")
        for name in self.robot_names:
            thread = threading.Thread(target=self._robot_worker, args=(name,), name=f"Worker-{name}", daemon=True)
            self.worker_threads.append(thread)
            thread.start()
        
        # --- Giai đoạn huấn luyện chính (Learner Loop) ---
        train_cfg = self.cfg_manager.train_cfg
        rospy.loginfo(f"Chuẩn bị vào vòng lặp huấn luyện Learner.")
        
        last_cycle = self.start_cycle - 1
        for cycle in range(self.start_cycle, train_cfg['epochs']):
            last_cycle = cycle
            rospy.loginfo(f"--- Bắt đầu Chu kỳ {cycle + 1}/{train_cfg['epochs']} ---")

            if self.sys_controller.check_for_signals() or rospy.is_shutdown():
                break

            self._collect_data_and_train(cycle)
        
        rospy.loginfo("Đã thoát khỏi vòng lặp huấn luyện chính.")
        self._shutdown(last_cycle)

    def _collect_data_and_train(self, cycle):
        """Thu thập dữ liệu từ queue, huấn luyện model, và log kết quả."""
        collected_steps = 0
        n_steps = self.cfg_manager.train_cfg.get('n_steps', 8192)
        
        cycle_total_rewards = {name: 0.0 for name in self.robot_names}
        cycle_avg_rewards = {name: 0.0 for name in self.robot_names}
        cycle_steps_taken = {name: 0 for name in self.robot_names}
        
        self.memory.clear()
        while collected_steps < n_steps:
            try:
                payload: WorkerPayload = self.experience_queue.get(timeout=20.0)
                
                robot_name, experience = payload.robot_name, payload.experience

                self.memory.add(*experience)
                
                # Tổng hợp số liệu để logging
                cycle_total_rewards[robot_name] += experience.reward
                cycle_steps_taken[robot_name] += 1
                collected_steps += 1
            except queue.Empty:
                rospy.logwarn("Hàng đợi kinh nghiệm trống trong 20s. Worker có thể bị kẹt hoặc đã dừng.")
                if rospy.is_shutdown() or self.stop_event.is_set(): return
                if not any(t.is_alive() for t in self.worker_threads):
                    rospy.logerr("Tất cả các luồng worker đã dừng. Thoát khỏi chu kỳ thu thập.")
                    return

        if len(self.memory) > 0:
            states_for_norm = np.array(self.memory.states)
            self.model_manager.obs_normalizer.observe(states_for_norm)
            self.trainer.update(self.memory)
        
        # Tính toán và Log
        system_total_reward = sum(cycle_total_rewards.values())
        total_steps_in_cycle = sum(cycle_steps_taken.values())
        system_avg_reward = system_total_reward / total_steps_in_cycle if total_steps_in_cycle > 0 else 0
        for name in self.robot_names:
            steps = cycle_steps_taken[name]
            cycle_avg_rewards[name] = cycle_total_rewards[name] / steps if steps > 0 else 0
        
        self._log_and_save(cycle, cycle_total_rewards, cycle_avg_rewards, system_total_reward, system_avg_reward)

    def _robot_worker(self, robot_name: str):
        """Vòng lặp tương tác với môi trường cho một robot cụ thể."""
        rospy.loginfo(f"✅ Worker cho [{robot_name}] đã bắt đầu.")
        env = self.envs[robot_name]
        steps_in_task = 0
        max_steps = self.cfg_manager.env_cfg['max_steps_per_episode']
        
        control_frequency = self.cfg_manager.env_cfg.get('control_frequency', 12)
        target_period = 1.0 / control_frequency

        try:
            while not self.stop_event.is_set():
                loop_start_time = time.time()

                observation_raw = env.lastest_obs
                if observation_raw is None:
                    rospy.logwarn_throttle(5, f"[{robot_name}] Observation thô là None, bỏ qua.")
                    rospy.sleep(0.1)
                    continue

                # --- BƯỚC KIỂM TRA DỮ LIỆU ĐẦU VÀO ---
                if np.isnan(observation_raw).any() or np.isinf(observation_raw).any():
                    rospy.logerr(f"[{robot_name}] LỖI: Observation THÔ chứa NaN/Inf! Bỏ qua.")
                    rospy.logerr(f"    Giá trị Obs thô: {observation_raw}")
                    continue

                observation_normalized = self.model_manager.obs_normalizer.normalize(np.array([observation_raw]))
                
                if np.isnan(observation_normalized).any() or np.isinf(observation_normalized).any():
                    rospy.logerr(f"[{robot_name}] LỖI: Observation ĐÃ CHUẨN HÓA chứa NaN/Inf! Bỏ qua.")
                    rospy.logerr(f"    Giá trị Obs chuẩn hóa: {observation_normalized}")
                    continue
                
                state_tensor = torch.FloatTensor(observation_normalized).to(self.model_manager.device)

                with torch.no_grad():
                    action, log_prob = self.model_manager.policy_net.get_action(state_tensor)
                
                action_np = action.cpu().numpy()[0]
                log_prob_np = log_prob.cpu().numpy()[0]
                
                env.step(action_np)
                
                reward, done, goal_reached, crashed_status, _ = env.calculate_reward_and_done(step=steps_in_task)
                
                timeout = steps_in_task >= max_steps - 1
                if timeout: reward -= 50
                is_episode_end = done or timeout
                reward = np.clip(reward, -100, 100)  # Giới hạn phần thưởng
                
                steps_in_task += 1
                
                if is_episode_end:
                    # === KHÔI PHỤC LOG CHI TIẾT ===
                    if goal_reached:
                        rospy.loginfo(f"🎉 [{robot_name}] ĐÃ TỚI ĐÍCH! Tổng bước: {steps_in_task}.reward: {reward:.2f}")
                    elif crashed_status:
                        rospy.logwarn(f"💥 [{robot_name}] VA CHẠM! Resetting model và episode. Tổng bước: {steps_in_task}.reward: {reward:.2f}")
                        self.reset_robot_model(robot_name)
                    elif timeout:
                        rospy.loginfo(f"⏰ [{robot_name}] HẾT THỜI GIAN. Tổng bước: {steps_in_task}.reward: {reward:.2f}")
                    elif done and steps_in_task >3: # Trường hợp khác (ví dụ: off_track)
                        rospy.logwarn(f"Ep [{robot_name}] kết thúc không thành công. Tổng bước: {steps_in_task}. reward: {reward:.2f}")
                    # =================================

                    self.request_and_set_new_path(robot_name)
                    env.reset_internal_state()
                    steps_in_task = 0
                if steps_in_task <= 2 :
                    continue
                exp = Experience(observation_normalized[0], action_np, reward, is_episode_end, log_prob_np)
                payload = WorkerPayload(robot_name, exp, goal_reached, crashed_status)
                self.experience_queue.put(payload)
                
                # --- Ngủ thông minh ---
                elapsed_time = time.time() - loop_start_time
                sleep_time = target_period - elapsed_time
                if sleep_time > 0:
                    rospy.sleep(sleep_time)
                else:
                    rospy.logwarn_throttle(10, f"[{robot_name}] Vòng lặp chạy chậm! Time: {elapsed_time:.4f}s > Target: {target_period:.4f}s")

        except Exception as e:
            rospy.logerr(f"🚨 Lỗi nghiêm trọng trong worker [{robot_name}]: {e}", exc_info=True)
        finally:
            rospy.loginfo(f"🛑 Worker cho [{robot_name}] đã dừng.")

    def request_and_set_new_path(self, robot_name, max_retries=3, retry_delay=1.0):
        """Yêu cầu path mới, với logic thử lại nếu thất bại."""
        for attempt in range(max_retries):
            try:
                env_to_set = self.envs[robot_name]
                point_list = [
                    Point(x=-17.7, y=14.6, z=0.0), Point(x=-17.7, y=-16.5, z=0.0),
                    Point(x=-5.1, y=-16.6, z=0.0), Point(x=10.2, y=-16.4, z=0.0),
                    Point(x=26.4, y=14.8, z=0.0),
                ]
                req = RequestPathRequest(robot_name=robot_name, end_point=random.choice(point_list))
                #req = ProvidePathRequest(robot_name=robot_name)
                response = self.RequestPath_client(req)
                
                if response.success and len(response.path.poses) > 1:
                    env_to_set.set_new_path(response.path)
                    return True
                else:
                    rospy.logwarn(f"[{robot_name}] Yêu cầu path thất bại: {response.message}. Thử lại sau {retry_delay}s...")
                    rospy.sleep(retry_delay)
            except Exception as e:
                rospy.logerr(f"[{robot_name}] Lỗi khi gọi service 'RequestPath': {e}. Thử lại sau {retry_delay}s...")
                rospy.sleep(retry_delay)
        rospy.logerr(f"[{robot_name}] Không thể lấy path hợp lệ sau {max_retries} lần thử.")
        return False

    def _log_and_save(self, cycle, total_rewards, avg_rewards, system_total, system_avg):
        """Hàm log và save được gọi từ Learner."""
        current_cycle = cycle + 1
        log_interval = self.cfg_manager.train_cfg.get('log_interval', 10)
        save_interval = self.cfg_manager.checkpoint_cfg.get('save_interval', 100)

        if current_cycle % log_interval == 0:
            rospy.loginfo("="*50)
            rospy.loginfo(f"Kết quả Chu kỳ (Cycle) số {current_cycle}:")
            
            self.logger.log_cycle_data(
                cycle_num=current_cycle,
                total_rewards_per_robot=total_rewards,
                avg_rewards_per_robot=avg_rewards,
                system_total_reward=system_total,
                system_avg_reward=system_avg
            )
            for name in self.robot_names:
                total_reward = total_rewards.get(name, 0)
                avg_reward = avg_rewards.get(name, 0)
                rospy.loginfo(f"  - {name}: Total Reward = {total_reward:.2f}, Avg Reward/Step = {avg_reward:.3f}")
            
            rospy.loginfo(f"--> Total System Reward: {system_total:.2f}")
            rospy.loginfo(f"--> Avg System Reward/Step: {system_avg:.3f}")
            rospy.loginfo("="*50)

        if current_cycle % save_interval == 0:
            self.model_manager.save_checkpoint(current_cycle)

    def _shutdown(self, last_cycle):
        """Dọn dẹp và kết thúc chương trình một cách an toàn."""
        rospy.loginfo("Bắt đầu quá trình shutdown...")
        self.stop_event.set()
        
        rospy.loginfo("Đang chờ các luồng worker dừng...")
        for thread in self.worker_threads:
            thread.join(timeout=5.0)
            if thread.is_alive():
                rospy.logwarn(f"Luồng {thread.name} không dừng đúng hạn.")

        self.model_manager.save_final_models(last_cycle + 1)
        
        for env in self.envs.values():
            env.close()
            
        rospy.loginfo("Hoàn tất shutdown.")

    def reset_all_models(self):
        """Reset vị trí của tất cả các model robot trong Gazebo."""
        for i, name in enumerate(self.robot_names):
            state = ModelState(model_name=name)
            base_x, offset = 21.5, 2.0
            state.pose.position.x = base_x + i * offset
            state.pose.position.y = 15.0
            quat = quaternion_from_euler(0, 0, np.pi / 2.0)  
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quat
            state.reference_frame = "world"
            try:
                self.set_model_state(state)
            except rospy.ServiceException as e:
                rospy.logerr(f"[{name}] Lỗi khi gọi reset model: {e}")

    def reset_robot_model(self, robot_name_to_reset):
        """Reset vị trí và trạng thái của một robot cụ thể trong Gazebo."""
        if robot_name_to_reset not in self.robot_names: return False
        state = ModelState(model_name=robot_name_to_reset)
        try:
            robot_index = self.robot_names.index(robot_name_to_reset)
            base_x, offset = 21.5, 2.0
            state.pose.position.x = base_x + robot_index * offset
            state.pose.position.y = 15.0
            quat = quaternion_from_euler(0, 0, np.pi / 2.0)
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quat
            state.reference_frame = "world"
            resp = self.set_model_state(state)
            if not resp.success:
                rospy.logwarn(f"[{robot_name_to_reset}] Reset model thất bại: {resp.status_message}")
            return resp.success
        except Exception as e:
            rospy.logerr(f"[{robot_name_to_reset}] Lỗi khi gọi service set_model_state: {e}")
            return False