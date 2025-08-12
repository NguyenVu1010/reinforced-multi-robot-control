#!/usr/bin/env python3

# nodes/multi_robot_runner.py

import rospy
import torch
import numpy as np
import os
import traceback
import threading
import pickle
import time

# --- Thêm thư mục gốc vào sys.path ---
# (Giữ lại nếu cần thiết cho việc import)
# ...

# --- Import các thành phần cần thiết ---
from models.policy import PolicyNetwork
from env.environment import MiRPathFollowingEnv 
from train.config_manager import ConfigManager
from utils.normalizer import ObservationNormalizer 
from nav_msgs.msg import Path
from std_msgs.msg import String

class MultiRobotRunner:
    """
    Tải mô hình AI đã huấn luyện và điều khiển đồng thời nhiều robot
    sử dụng kiến trúc đa luồng, phi tập trung, tương thích với file train.
    """
    def __init__(self, robot_names, model_path, config_path, normalizer_path):
        self.robot_names = robot_names
        
        # --- 1. Tải Config và Khởi tạo Môi trường ---
        rospy.loginfo("Đang tải file cấu hình...")
        self.config_manager = ConfigManager(config_path=config_path)
        
        rospy.loginfo(f"Đang khởi tạo {len(robot_names)} môi trường con...")
        # Mỗi môi trường sẽ tự subscribe vào /rl_observation
        self.envs = {name: MiRPathFollowingEnv(name, robot_names, self.config_manager) for name in self.robot_names}
        
        # --- 2. Trạng thái và điều khiển luồng ---
        self.goal_reached_flags = {name: True for name in self.robot_names}
        self.control_threads = {}
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        # --- 3. Tải Model và Normalizer ---
        rospy.loginfo("Đang tải mô hình AI và normalizer...")
        self._load_assets(model_path, normalizer_path)

        # --- 4. Thiết lập ROS ---
        self.path_subs = {
            name: rospy.Subscriber(f'/paths/{name}/planned_path', Path, self.path_callback, callback_args=name) 
            for name in self.robot_names
        }
        self.completion_pub = rospy.Publisher('/task_completion', String, queue_size=10)

    def _load_assets(self, model_path, normalizer_path):
        """Hàm helper để tải model và normalizer."""
        if not os.path.exists(model_path): raise FileNotFoundError(f"Model not found: {model_path}")
        if not os.path.exists(normalizer_path): raise FileNotFoundError(f"Normalizer not found: {normalizer_path}")
        
        # Lấy thông tin từ config để khởi tạo model
        # Giả sử action_space đã được định nghĩa trong Env và ModelManager dùng config
        env_cfg = self.config_manager.env_cfg
        model_cfg = self.config_manager.model_cfg
        
        with open(normalizer_path, 'rb') as f:
            self.obs_normalizer = pickle.load(f)
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.policy_net = PolicyNetwork(
            state_dim=env_cfg['state_dim'], 
            action_dim=env_cfg['action_dim'], 
            hidden_dim=model_cfg['hidden_dim']
        ).to(self.device)
        
        self.policy_net.load_state_dict(torch.load(model_path, map_location=self.device))
        self.policy_net.eval() # QUAN TRỌNG: Chuyển sang chế độ Inference
        rospy.loginfo(f"Đã tải tất cả assets lên thiết bị '{self.device}'.")

    def path_callback(self, msg, robot_name):
        """Callback khi một robot nhận được quỹ đạo mới."""
        with self.lock:
            if self.goal_reached_flags[robot_name]:
                # Env tự xử lý việc set_path và reset_internal_state
                self.envs[robot_name].set_new_path(msg) 
                self.goal_reached_flags[robot_name] = False
                rospy.loginfo(f"Runner: Đã nhận quỹ đạo mới cho '{robot_name}'. Bắt đầu luồng điều khiển.")
                
                # Khởi chạy luồng điều khiển
                if robot_name not in self.control_threads or not self.control_threads[robot_name].is_alive():
                    thread = threading.Thread(target=self._control_loop, args=(robot_name,), daemon=True)
                    self.control_threads[robot_name] = thread
                    thread.start()

    def _control_loop(self, robot_name: str):
        """
        Vòng lặp điều khiển riêng cho một robot. Chạy trong một luồng riêng.
        """
        rospy.loginfo(f"🚀 Luồng điều khiển cho [{robot_name}] đã bắt đầu.")
        env = self.envs[robot_name]
        
        control_frequency = self.config_manager.env_cfg.get('control_frequency', 12)
        rate = rospy.Rate(control_frequency)
        
        # Vòng lặp sẽ chạy cho đến khi cờ done được set
        while not self.goal_reached_flags[robot_name] and not self.stop_event.is_set():
            # 1. Lấy observation từ môi trường (môi trường đã tự subscribe)
            # Hàm này sẽ block nhẹ cho đến khi có obs mới
            observation_raw = env.lastest_obs
            if observation_raw is None:
                rospy.logwarn_throttle(5, f"[{robot_name}] Chưa nhận được observation, bỏ qua vòng lặp.")
                rate.sleep()
                continue

            # 2. Chuẩn hóa và quyết định hành động
            obs_normalized = self.obs_normalizer.normalize(np.array([observation_raw]))
            state_tensor = torch.FloatTensor(obs_normalized).to(self.device)
            
            with torch.no_grad():
                # --- THAY ĐỔI QUAN TRỌNG KHI INFERENCE ---
                # Lấy hành động quyết định (mean), không lấy mẫu (sample)
                dist = self.policy_net.forward(state_tensor)
                action_tensor = dist.mean
                # Không cần clip ở đây nữa vì policy đã có tanh + scale
                action_np = action_tensor.cpu().numpy()[0]
            # 3. Thực thi hành động
            env.step(action_np)

            # 4. Kiểm tra điều kiện kết thúc từ môi trường
            # `obs_vector` được truyền vào là obs_raw của bước này
            _, done, goal_reached, crashed, _ = env.calculate_reward_and_done( step=env.current_timestep)

            if crashed:
                rospy.logerr(f"💥 [{robot_name}] ĐÃ VA CHẠM! Dừng nhiệm vụ.")
                self.completion_pub.publish(String(data=f"{robot_name}_crashed"))
                break # Thoát khỏi vòng lặp while
            
            if goal_reached:
                rospy.loginfo(f"🎉 [{robot_name}] ĐÃ TỚI ĐÍCH!")
                self.completion_pub.publish(String(data=f"{robot_name}_completed"))
                break # Thoát khỏi vòng lặp while

            # 5. Duy trì tần số
            rate.sleep()
        
        # Khi vòng lặp kết thúc (tới đích, va chạm, hoặc node dừng)
        with self.lock:
            self.goal_reached_flags[robot_name] = True
        env.close() # Gửi lệnh vận tốc bằng 0
        rospy.loginfo(f"🛑 Luồng điều khiển cho [{robot_name}] đã kết thúc.")


    def run(self):
        """Bắt đầu node và giữ cho nó hoạt động."""
        rospy.loginfo("Robot Runner (Phi tập trung) đã sẵn sàng. Chờ nhận nhiệm vụ từ Task Manager...")
        rospy.spin()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Bắt đầu quá trình shutdown...")
        self.stop_event.set()
        for env in self.envs.values():
            env.close()
        rospy.loginfo("Runner đã dừng.")

if __name__ == '__main__':
    runner = None
    try:
        rospy.init_node('multi_robot_runner_node')
        
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mir_control') # !!! THAY TÊN PACKAGE CỦA BẠN !!!

        model_file = rospy.get_param('~model_file', 'policy_net_final.pth')
        config_file_name = rospy.get_param('~config_file_name', 'config.yaml')
        normalizer_file = rospy.get_param('~normalizer_file', 'obs_normalizer.pkl')
        
        MODEL_PATH = os.path.join(package_path, 'saved_models', model_file)
        CONFIG_PATH = os.path.join(package_path, 'config', config_file_name)
        NORMALIZER_PATH = os.path.join(package_path, 'saved_models', normalizer_file)
        
        ROBOT_NAMES = ['robot1', 'robot2', 'robot3']

        rospy.loginfo(f"Đang tải model từ: {MODEL_PATH}")
        rospy.loginfo(f"Đang tải config từ: {CONFIG_PATH}")
        rospy.loginfo(f"Đang tải normalizer từ: {NORMALIZER_PATH}")

        runner = MultiRobotRunner(
            robot_names=ROBOT_NAMES, 
            model_path=MODEL_PATH, 
            config_path=CONFIG_PATH,
            normalizer_path=NORMALIZER_PATH
        )
        runner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Runner kết thúc do ROS interrupt.")
    except Exception as e:
        rospy.logerr(f"Runner gặp lỗi nghiêm trọng: {e}")
        traceback.print_exc()
    finally:
        if runner and not rospy.is_shutdown():
            runner.shutdown()