#!/usr/bin/env python3

import rospy
import torch
import yaml
import numpy as np
import os
import traceback
import threading
import pickle

# --- Thêm thư mục gốc vào sys.path để đảm bảo import hoạt động ---
import sys
# Lấy đường dẫn đến thư mục chứa file script này (RUN/)
script_dir = os.path.dirname(os.path.abspath(__file__))
# Đi ngược lên 1 cấp để đến thư mục gốc của package (mir_control)
project_root = os.path.abspath(os.path.join(script_dir, '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
# -----------------------------------------------------------------

# Import các thành phần của dự án từ các vị trí mới
from models.policy import PolicyNetwork
from env.environment import MiRPathFollowingEnv 
from train.config_manager import ConfigManager # Import từ thư mục train
from utils.normalizer import ObservationNormalizer 

# Import các kiểu message ROS
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

class MultiRobotRunner:
    """
    Tải mô hình AI đã huấn luyện và điều khiển đồng thời nhiều robot
    để bám theo các quỹ đạo nhận được từ Task Manager.
    """
    def __init__(self, robot_names, model_path, config_path, normalizer_path):
        self.robot_names = robot_names
        self.lock = threading.Lock()
        self.latest_model_states = None
        self.steps_in_task = {name: 0 for name in self.robot_names}
        # --- 1. Tải Config Manager và Khởi tạo Môi trường ---
        rospy.loginfo("Đang tải file cấu hình...")
        self.config_manager = ConfigManager(config_path=config_path)
        
        rospy.loginfo("Đang khởi tạo các môi trường con...")
        # Truyền config_manager vào cho mỗi môi trường
        self.envs = {name: MiRPathFollowingEnv(name, robot_names, self.config_manager) for name in self.robot_names}
        
        # --- 2. Trạng thái của Runner ---
        self.goal_reached_flags = {name: True for name in self.robot_names}
        self.odom_received = False

        # --- 3. Tải Model và Normalizer ---
        rospy.loginfo("Đang tải mô hình AI và normalizer...")
        self._load_assets(model_path, normalizer_path)

        # --- 4. Thiết lập ROS ---
        self.cmd_pubs = {name: rospy.Publisher(f'/{name}/mobile_base_controller/cmd_vel', Twist, queue_size=1) for name in self.robot_names}
        self.path_subs = {name: rospy.Subscriber(f'/paths/{name}/planned_path', Path, self.path_callback, callback_args=name) for name in self.robot_names}
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.completion_pub = rospy.Publisher('/task_completion', String, queue_size=10)
        
        self.rate = rospy.Rate(20) # Tần số điều khiển

    def _load_assets(self, model_path, normalizer_path):
        """Hàm helper để tải model và normalizer."""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Không tìm thấy file model tại: {model_path}")
        if not os.path.exists(normalizer_path):
            raise FileNotFoundError(f"Không tìm thấy file normalizer tại: {normalizer_path}")
        
        # Lấy các dim từ config_manager
        env_cfg = self.config_manager.env_cfg
        model_cfg = self.config_manager.model_cfg
        
        # Tải normalizer
        with open(normalizer_path, 'rb') as f:
            self.obs_normalizer = pickle.load(f)
        
        # Tải model
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = PolicyNetwork(env_cfg['state_dim'], env_cfg['action_dim'], model_cfg['hidden_dim']).to(self.device)
        self.policy_net.load_state_dict(torch.load(model_path, map_location=self.device))
        self.policy_net.eval() # Quan trọng: Chuyển sang chế độ Inference
        rospy.loginfo(f"Đã tải tất cả assets lên thiết bị '{self.device}'.")

    def path_callback(self, msg, robot_name):
        """Callback khi một robot nhận được quỹ đạo mới từ Task Manager."""
        with self.lock:
            # Gán path vào env tương ứng, env sẽ lo phần còn lại
            self.envs[robot_name].set_new_path(msg)
            self.goal_reached_flags[robot_name] = False
            rospy.loginfo(f"Runner: Đã nhận quỹ đạo mới cho '{robot_name}'.")

    def model_states_callback(self, msg):
        self.latest_model_states = msg

    def run(self):
        """Bắt đầu vòng lặp điều khiển chính."""
        rospy.loginfo("Chờ odometry từ Gazebo...")
        while self.latest_model_states is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
        if rospy.is_shutdown(): return
        
        rospy.loginfo("Robot Runner đã sẵn sàng. Chờ nhận nhiệm vụ từ Task Manager...")

        while not rospy.is_shutdown():
            active_robot_names, observations_raw = [], []
            with self.lock:
                for name in self.robot_names:
                    if not self.goal_reached_flags[name]:
                        # Lấy quan sát "thô" từ môi trường
                        observations_raw.append(self.envs[name]._get_obs())
                        active_robot_names.append(name)
            
            if active_robot_names:
                # Chuẩn hóa quan sát trước khi đưa vào mạng
                obs_normalized = self.obs_normalizer.normalize(np.array(observations_raw))
                states_tensor = torch.FloatTensor(obs_normalized).to(self.device)
                
                with torch.no_grad():
                    # Lấy hành động quyết định (mean) từ policy
                    actions = self.policy_net(states_tensor).mean
                actions_np = actions.cpu().numpy()

                with self.lock:
                    for i, name in enumerate(active_robot_names):
                        # Gửi lệnh điều khiển
                        self.envs[name].step(actions_np[i],observations_raw[i][1])
                        
                        # Kiểm tra xem robot này đã tới đích chưa
                        env = self.envs[name]
                        reward, done, goal_reached,crashed , min_obstacle_dist= env.calculate_reward_and_done(self.latest_model_states,step=self.steps_in_task[name])
                        if goal_reached:
                            rospy.loginfo(f"'{name}' đã tới đích!")
                            self.goal_reached_flags[name] = True
                            self.cmd_pubs[name].publish(Twist())
                            self.completion_pub.publish(String(data=name))
                            env.reset_internal_state()
                            self.steps_in_task[name] = 0
                        self.steps_in_task[name] += 1
            
            self.rate.sleep()
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("Đang dừng tất cả robot...")
        for pub in self.cmd_pubs.values():
            pub.publish(Twist())

if __name__ == '__main__':
    runner = None
    try:
        rospy.init_node('multi_robot_runner_node')
        
        # Lấy đường dẫn gốc của package để xây dựng các đường dẫn file một cách đáng tin cậy
        import rospkg
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mir_control') # !!! THAY TÊN PACKAGE CỦA BẠN VÀO ĐÂY !!!

        # Lấy các tham số từ ROS Param Server (được truyền từ file launch)
        model_file = rospy.get_param('~model_file', 'policy_net_final.pth')
        config_file_name = rospy.get_param('~config_file_name', 'config.yaml')
        normalizer_file = rospy.get_param('~normalizer_file', 'obs_normalizer.pkl')
        
        # Xây dựng các đường dẫn tuyệt đối
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
        rospy.loginfo("Runner kết thúc.")
    except Exception as e:
        rospy.logerr(f"Runner gặp lỗi nghiêm trọng: {e}")
        traceback.print_exc()
    finally:
        if runner:
            runner.shutdown()