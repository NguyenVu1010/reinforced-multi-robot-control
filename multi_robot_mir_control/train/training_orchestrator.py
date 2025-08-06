#!/usr/bin/env python3

# trainer_logic/training_orchestrator.py

import rospy
import torch
import numpy as np
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point
import random
from tf.transformations import quaternion_from_euler
# Giả định các lớp này đã được import đúng cách từ các package tương ứng
# Sửa lại đường dẫn import cho đúng với cấu trúc package của bạn
from env.environment import MiRPathFollowingEnv 
from core.memory import Memory
from core.trainer import PPOTrainer
from mir_control.srv import RequestPath, RequestPathRequest
from gazebo_msgs.msg import ModelStates
from utils.training_logger import TrainingLogger
class TrainingOrchestrator:
    """
    Điều phối toàn bộ vòng lặp huấn luyện, tái tạo chính xác logic gốc,
    với một vòng lặp tuần tự qua nhiều instance môi trường.
    """
    def __init__(self, config_manager, model_manager, system_controller):
        self.cfg_manager = config_manager
        self.model_manager = model_manager
        self.sys_controller = system_controller
        
        self.num_robots = self.cfg_manager.env_cfg['num_robots']
        self.robot_names = [f'robot{i+1}' for i in range(self.num_robots)]
        self.last_cycle_total_rewards_per_robot = {name: 0.0 for name in self.robot_names}
        # --- Khởi tạo NHIỀU instance môi trường ---
        rospy.loginfo(f"Khởi tạo {self.num_robots} môi trường Gym cho: {self.robot_names}")
        self.envs = {
            name: MiRPathFollowingEnv(name, self.robot_names, self.cfg_manager) 
            for name in self.robot_names
        }
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        # --- Khởi tạo các thành phần khác ---
        ppo_trainer_params = self.cfg_manager.train_cfg.copy()
        ppo_trainer_params.pop('epochs', None)
        ppo_trainer_params.pop('log_interval', None)
        self.logger = TrainingLogger(self.model_manager.save_dir, self.robot_names,self.cfg_manager.checkpoint_cfg['log_filename'])
        self.trainer = PPOTrainer(
            policy_net=self.model_manager.policy_net, 
            value_net=self.model_manager.value_net, 
            **ppo_trainer_params
        )
        self.memory = Memory()
        self.steps_in_task = {name: 0 for name in self.robot_names}
        self.start_cycle = 0
        self.latest_model_states = None

        # Các biến để logging
        self.last_cycle_avg_reward_per_robot = {name: 0.0 for name in self.robot_names}
        self.last_cycle_total_reward_per_robot = {name: 0.0 for name in self.robot_names}
        self.last_cycle_avg_system_reward = 0.0
        
        # Kết nối service và subscriber
        self._connect_ros_infra()
        self.model_states_sub = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self._model_states_callback, queue_size=1
        )
        
    def _model_states_callback(self, msg):
        self.latest_model_states = msg

    def _connect_ros_infra(self):
        rospy.loginfo("Đang chờ service 'request_path_service'...")
        self.service_name = 'request_path_service'
        try:
            rospy.wait_for_service(self.service_name, timeout=30.0)
            self.RequestPath_client = rospy.ServiceProxy(self.service_name, RequestPath)
        except rospy.ROSException as e:
            raise RuntimeError(f"Không thể kết nối service 'request_path_service': {e}")

    # ===================================================================
    # HÀM RUN
    # ===================================================================
    def run(self):
        """Hàm chạy vòng lặp huấn luyện chính."""
        self.sys_controller.cleanup_control_files()
        self.start_cycle = self.model_manager.load_checkpoints()

        rospy.loginfo("Chờ message đầu tiên từ /gazebo/model_states...")
        while self.latest_model_states is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown(): return
        
        # --- Giai đoạn thiết lập ban đầu ---
        rospy.loginfo("Yêu cầu các quỹ đạo ban đầu...")
        all_paths_ok = True
        for name in self.robot_names:
            if not self.request_and_set_new_path(name):
                all_paths_ok = False
        
        if not all_paths_ok:
            rospy.logfatal("Không thể lấy được quỹ đạo ban đầu cho tất cả robot. Dừng chương trình.")
            return # Dừng lại nếu không thể bắt đầu

        rospy.loginfo("Đã thiết lập path ban đầu. Reset môi trường...")
        for env in self.envs.values():
            env.reset()
        rospy.loginfo("Reset môi trường hoàn tất.")

        # --- Giai đoạn huấn luyện chính ---
        train_cfg = self.cfg_manager.train_cfg
        rospy.loginfo(f"Chuẩn bị vào vòng lặp huấn luyện. "
                      f"start_cycle = {self.start_cycle}, epochs = {train_cfg['epochs']}")
        
        last_cycle = self.start_cycle - 1 # Khởi tạo giá trị an toàn
        for cycle in range(self.start_cycle, train_cfg['epochs']):
            last_cycle = cycle
            rospy.loginfo(f"--- Bắt đầu Chu kỳ {cycle + 1}/{train_cfg['epochs']} ---")
            self.reset_all_models()
            if self.sys_controller.check_for_signals() or rospy.is_shutdown():
                break

            self._collect_trajectories()
            
            if len(self.memory) > 0:
                self.trainer.update(self.memory)
            self.memory.clear()
            self._log_and_save(cycle)
        
        rospy.loginfo("Đã thoát khỏi vòng lặp huấn luyện chính.")
        self._shutdown(last_cycle)

    # ===================================================================
    # CÁC HÀM HELPER KHÁC
    # ===================================================================

    def _collect_trajectories(self):
        max_steps = self.cfg_manager.env_cfg['max_steps_per_episode']
        cycle_rewards = {name: 0.0 for name in self.robot_names}
        cycle_steps = {name: 0 for name in self.robot_names}
        
        for _ in range(max_steps):
            if self.latest_model_states is None or rospy.is_shutdown(): break
            
            current_model_states = self.latest_model_states
            observations_raw = [env.update_state_and_get_obs(current_model_states) for env in self.envs.values()]
            
            observations_np = np.array(observations_raw)
            for obs in observations_np:
                self.model_manager.obs_normalizer.observe(obs)
            observations_normalized = self.model_manager.obs_normalizer.normalize(observations_np)
            states_tensor = torch.FloatTensor(observations_normalized).to(self.model_manager.device)

            with torch.no_grad():
                actions, log_probs = self.model_manager.policy_net.get_action(states_tensor)
            
            actions_np = actions.cpu().numpy()
            log_probs_np = log_probs.cpu().numpy()
            
            for i, name in enumerate(self.robot_names):
                self.envs[name].step(actions_np[i],observations_raw[i][1])
            
            if self.envs: next(iter(self.envs.values())).rate.sleep()

            next_model_states = self.latest_model_states
            if next_model_states is None: break

            for i, name in enumerate(self.robot_names):
                env = self.envs[name]
                reward, done, goal_reached,crashed_status , min_obstacle_dist= env.calculate_reward_and_done(next_model_states,step=self.steps_in_task[name])
                
                # --- LOGIC XỬ LÝ KẾT THÚC EPISODE ĐÚNG ĐẮN ---
                timeout = self.steps_in_task[name] >= max_steps - 1 # -1 để an toàn
                if timeout:
                    reward += -100

                is_episode_end = done or timeout 
                
                self.memory.add(observations_normalized[i], actions_np[i], reward, is_episode_end, log_probs_np[i])
                
                self.steps_in_task[name] += 1
                cycle_rewards[name] += reward
                cycle_steps[name] += 1
                
                if is_episode_end:
                    if goal_reached:
                        rospy.loginfo(f"🎉 [{name}] ĐÃ TỚI ĐÍCH! Reward cuối: {reward:.2f}, Tổng bước: {self.steps_in_task[name]}.")
                    elif crashed_status:
                        self.reset_robot_model(name) 
                        rospy.logwarn(f"💥 [{name}] Va chạm & Không thoát được! Reward cuối: {reward:.2f}, Dist: {min_obstacle_dist:.2f}, Bị kẹt trong {self.steps_in_task[name]} bước.")
                    # elif done and not goal_reached:
                    #     rospy.logwarn(f"💥 [{name}] Episode thất bại. Reward cuối: {reward:.2f}, Tổng bước: {self.steps_in_task[name]}.")
                    elif timeout:
                        rospy.loginfo(f"⏰ [{name}] Hết thời gian. Reward cuối: {reward:.2f}, Tổng bước: {self.steps_in_task[name]}.")
                    
                    self.request_and_set_new_path(name)
                    env.reset_internal_state()
                    self.steps_in_task[name] = 0
        
        # --- CẬP NHẬT BIẾN LOGGING (Đã sửa) ---
        total_steps = sum(cycle_steps.values())
        self.last_cycle_avg_system_reward = sum(cycle_rewards.values()) / total_steps if total_steps > 0 else 0
        
        for name in self.robot_names:
            steps = cycle_steps[name]
            total_r = cycle_rewards[name]
            self.last_cycle_total_reward_per_robot[name] = total_r
            self.last_cycle_avg_reward_per_robot[name] = total_r / steps if steps > 0 else 0
    def request_and_set_new_path(self, robot_name, max_retries=3, retry_delay=1.0):
        """Yêu cầu path mới, với logic thử lại nếu thất bại."""
        for attempt in range(max_retries):
            try:
                env_to_set = self.envs[robot_name]
                point_list = [
                    Point(x=-17.7, y=14.6, z=0.0),
                    Point(x=-17.7, y=-16.5, z=0.0),
                    Point(x=-5.1, y=-16.6, z=0.0),
                    Point(x=10.2, y=-16.4, z=0.0),
                    Point(x=26.4, y=14.8, z=0.0),
                ]
                req = RequestPathRequest()
                req.robot_name = robot_name
                req.end_point = random.choice(point_list)


                response = self.RequestPath_client(req)
                
                if response.success and len(response.path.poses) > 1:
                    #rospy.loginfo(f"[{robot_name}] Nhận được path thành công. với {len(response.path.poses)} điểm.")
                    env_to_set.set_new_path(response.path)
                    return True
                else:
                    rospy.logwarn(f"[{robot_name}] Yêu cầu path thất bại hoặc path rỗng: {response.message}. Thử lại sau {retry_delay}s.")
                    rospy.sleep(retry_delay)

            except rospy.ServiceException as e:
                rospy.logerr(f"[{robot_name}] Lỗi gọi service 'RequestPath': {e}. Thử lại sau {retry_delay}s.")
                rospy.sleep(retry_delay)
            except KeyError:
                rospy.logerr(f"Lỗi nghiêm trọng: Không tìm thấy môi trường cho robot tên '{robot_name}'")
                return False

        rospy.logerr(f"[{robot_name}] Không thể lấy được path hợp lệ sau {max_retries} lần thử.")
        return False

    def _log_and_save(self, cycle):
        current_cycle = cycle + 1
        log_interval = self.cfg_manager.train_cfg.get('log_interval', 10)
        save_interval = self.cfg_manager.checkpoint_cfg.get('save_interval', 100)

        if current_cycle % log_interval == 0:
            rospy.loginfo("="*50)
            rospy.loginfo(f"Kết quả Chu kỳ (Cycle) số {current_cycle}:")
            system_total_reward = sum(self.last_cycle_total_reward_per_robot.values())
            
            self.logger.log_cycle_data(
                cycle_num=current_cycle,
                total_rewards_per_robot=self.last_cycle_total_reward_per_robot,
                avg_rewards_per_robot=self.last_cycle_avg_reward_per_robot,
                system_total_reward=system_total_reward,
                system_avg_reward=self.last_cycle_avg_system_reward
            )
            for name in self.robot_names:
                total_reward = self.last_cycle_total_reward_per_robot.get(name, 0)
                avg_reward = self.last_cycle_avg_reward_per_robot.get(name, 0)
                rospy.loginfo(f"  - {name}: Total Reward = {total_reward:.2f}, Avg Reward/Step = {avg_reward:.3f}")
            
            total_system_reward = sum(self.last_cycle_total_reward_per_robot.values())
            rospy.loginfo(f"--> Total System Reward: {total_system_reward:.2f}")
            rospy.loginfo(f"--> Avg System Reward/Step: {self.last_cycle_avg_system_reward:.3f}")
            rospy.loginfo("="*50)

        if current_cycle % save_interval == 0:
            self.model_manager.save_checkpoint(current_cycle)
            

    def _shutdown(self, last_cycle):
        rospy.loginfo("Đang dọn dẹp và kết thúc...")
        self.model_manager.save_final_models(last_cycle + 1)
        for env in self.envs.values():
            env.close()
        rospy.loginfo("Hoàn tất.")
    def reset_all_models(self):

        for i, name in enumerate(self.robot_names):
            state = ModelState()
            state.model_name = name
            
            # Mỗi robot lệch ra theo trục x, tránh trùng nhau
            base_x = 21.5
            offset = 2.0  # khoảng cách giữa các robot
            state.pose.position.x = base_x + i * offset
            state.pose.position.y = 15.0
            state.pose.position.z = 0.0

            # Hướng robot theo trục x
            quat = quaternion_from_euler(0, 0, 0.0)
            state.pose.orientation.x = quat[0]
            state.pose.orientation.y = quat[1]
            state.pose.orientation.z = quat[2]
            state.pose.orientation.w = quat[3]

            # Không có vận tốc ban đầu
            state.twist.linear.x = 0.0
            state.twist.angular.z = 0.0
            state.reference_frame = "world"

            try:
                resp = self.set_model_state(state)
                if resp.success:
                    rospy.loginfo(f"[{name}] ✅ Reset model thành công.")
                else:
                    rospy.logwarn(f"[{name}] ❌ Reset model thất bại: {resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"[{name}] 🚨 Lỗi khi gọi reset model: {e}")
    def reset_robot_model(self, robot_name_to_reset):
        """
        Reset vị trí và trạng thái của một robot cụ thể trong Gazebo.

        Args:
            robot_name_to_reset (str): Tên của model robot cần reset.
        """
        
        # Kiểm tra xem robot có tồn tại trong danh sách quản lý không
        if robot_name_to_reset not in self.robot_names:
            rospy.logerr(f"🚨 Tên robot '{robot_name_to_reset}' không có trong danh sách quản lý. Không thể reset.")
            return False

        state = ModelState()
        state.model_name = robot_name_to_reset

        # --- Logic xác định vị trí reset ---
        # Ở đây, chúng ta vẫn dùng logic cũ dựa trên chỉ số (index) của robot
        # để đảm bảo mỗi robot luôn được reset về đúng vị trí được gán cho nó,
        # tránh việc reset chồng lên nhau.
        try:
            robot_index = self.robot_names.index(robot_name_to_reset)
        except ValueError:
            # Trường hợp này đã được kiểm tra ở trên, nhưng để cho chắc chắn
            rospy.logerr(f"Lỗi không mong muốn: không tìm thấy index cho '{robot_name_to_reset}'.")
            return False

        base_x = 21.5
        offset = 2.0  # khoảng cách giữa các vị trí reset của robot
        state.pose.position.x = base_x + robot_index * offset
        state.pose.position.y = 15.0
        state.pose.position.z = 0.0

        # Hướng robot về phía trước (dọc theo trục x)
        quat = quaternion_from_euler(0, 0, 0.0)
        state.pose.orientation.x = quat[0]
        state.pose.orientation.y = quat[1]
        state.pose.orientation.z = quat[2]
        state.pose.orientation.w = quat[3]

        # Đặt vận tốc về 0
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        
        state.reference_frame = "world"

        try:
            # Sử dụng service proxy đã được khởi tạo một lần
            resp = self.set_model_state(state)
            if resp.success:
                rospy.loginfo(f"[{robot_name_to_reset}] ✅ Reset model thành công về vị trí ban đầu.")
                return True
            else:
                rospy.logwarn(f"[{robot_name_to_reset}] ❌ Reset model thất bại: {resp.status_message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"[{robot_name_to_reset}] 🚨 Lỗi khi gọi service set_model_state: {e}")
            return False