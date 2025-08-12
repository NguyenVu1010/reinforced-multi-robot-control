#!/usr/bin/env python3

# src/mir_gym_env/environment.py

import rospy
import gym
from gym import spaces
import numpy as np

# Import các thành phần logic (giả sử chúng nằm trong thư mục con)
from .include.robot_state import RobotState
from .include.robot_controller import RobotController
from .include.reward_calculator import RewardCalculator

# Import các kiểu message ROS
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from mir_control.msg import Observation
from std_msgs.msg import UInt16
from tf.transformations import euler_from_quaternion
class MiRPathFollowingEnv(gym.Env):
    """
    Môi trường Gym cho MỘT robot MiR, nhưng nhận thức được các robot khác.
    Mỗi robot sẽ có một instance riêng của lớp này.
    """
    def __init__(self, robot_name, all_robot_names, config_manager):
        super(MiRPathFollowingEnv, self).__init__()
        
        # --- THÔNG TIN CƠ BẢN ---
        self.robot_name = robot_name
        self.other_robot_names = [name for name in all_robot_names if name != self.robot_name]
        self.cfg = config_manager.env_cfg

        # --- CÁC THÀNH PHẦN CHO CHỈ ROBOT NÀY ---
        self.state_manager = RobotState() # Không cần truyền tên, vì nó chỉ quản lý 1 trạng thái
        self.controller = RobotController(robot_name)
        self.reward_calculator = RewardCalculator(robot_name)

        self.max_v = self.cfg.get('max_linear_velocity', 1.0)
        self.max_w = self.cfg.get('max_angular_velocity', 0.8)
        # --- BIẾN TRẠNG THÁI ---
        self.last_action = np.array([0.0, 0.0])
        self.other_robots_states_cache = {}
        self.latest_model_states = None
        self.current_timestep = 0
        self.lastest_obs = None  # Dữ liệu observation mới nhất từ ROS
        # Biến để theo dõi timestep
        self.max_steps = self.cfg['max_steps_per_episode']
        
        # --- ROS ---
        # Chỉ cần 1 subscriber cho robot này
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._model_states_callback, queue_size=1)
        self.obs_sub = rospy.Subscriber(f'/{self.robot_name}/rl_observation', Observation, self.update_obs, queue_size=1)
        self.nearest_idx_sub = rospy.Subscriber(f'/{self.robot_name}/nearest_idx',UInt16, self.update_nearest_idx, queue_size=1)
        self.path_sub = rospy.Subscriber(f'/paths/{self.robot_name}/planned_path', Path, self._path_callback, queue_size=1)
        self.rate = rospy.Rate(12)        
        self._wait_for_initial_data()
    def _path_callback(self, msg):
        self.set_new_path(msg)
    def update_nearest_idx(self, msg):
        self.state_manager.update_last_nearest_idx(msg.data)
    def update_obs(self, msg):
        self.lastest_obs = np.array(msg.data, dtype=np.float32)
    def _wait_for_initial_data(self):
        """
        Tạm dừng việc khởi tạo cho đến khi nhận được các message đầu tiên
        từ các topic quan trọng. Điều này đảm bảo môi trường không hoạt động
        với dữ liệu rỗng (None).
        """
        rospy.loginfo(f"[{self.robot_name}] Đang chờ dữ liệu ban đầu từ các topic...")
        
        # Vòng lặp sẽ tiếp tục cho đến khi tất cả các biến state cần thiết
        # không còn là None nữa.
        # Hãy tùy chỉnh điều kiện này cho phù hợp với các biến state của bạn.
        while (self.latest_model_states is None ) and not rospy.is_shutdown():
            rospy.logwarn_throttle(5, f"[{self.robot_name}] Vẫn đang chờ dữ liệu...")
            rospy.sleep(0.2)
            
        if not rospy.is_shutdown():
            rospy.loginfo(f"[{self.robot_name}] Đã nhận dữ liệu ban đầu. Môi trường sẵn sàng.")

    def _model_states_callback(self, msg):
        # Callback này chạy trên luồng của ROS, cập nhật state chung
        # Nó thread-safe vì việc gán trong Python là một hành động nguyên tử (atomic)
        self.latest_model_states = msg

    def update_state(self, model_states_msg):
        """
        Cập nhật trạng thái của robot này và cache trạng thái của các robot khác
        từ message /gazebo/model_states chung.
        *** NEW: Thêm kiểm tra lật (tilt check). ***
        """
        try:
            # Cập nhật cho chính mình
            my_idx = model_states_msg.name.index(self.robot_name)
            my_pose_msg = model_states_msg.pose[my_idx]
            my_twist_msg = model_states_msg.twist[my_idx]

            # --- NEW: LOGIC KIỂM TRA LẬT ---
            orientation_q = my_pose_msg.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            
            # Đặt ngưỡng lật, ví dụ 60 độ (pi/3 radian)
            tilt_threshold = np.pi / 3 
            is_tilted = abs(roll) > tilt_threshold or abs(pitch) > tilt_threshold
            
            if is_tilted:
                 rospy.logwarn_throttle(5, f"[{self.robot_name}] Phát hiện robot bị lật! Roll: {np.degrees(roll):.1f}°, Pitch: {np.degrees(pitch):.1f}°")

            # Cập nhật state manager với thông tin mới
            self.state_manager.update_kinematics(my_pose_msg, my_twist_msg, is_tilted)
            # --- KẾT THÚC LOGIC KIỂM TRA LẬT ---

            # Cache trạng thái của các robot khác (không thay đổi)
            temp_other_states = {}
            for name in self.other_robot_names:
                idx = model_states_msg.name.index(name)
                pose_msg, twist_msg = model_states_msg.pose[idx], model_states_msg.twist[idx]
                temp_other_states[name] = {
                    'pos': np.array([pose_msg.position.x, pose_msg.position.y]),
                    'vel': np.array([twist_msg.linear.x, twist_msg.linear.y])
                }
            self.other_robots_states_cache = temp_other_states
        except (ValueError, IndexError):
            pass

    def step(self, action):
        """API cho Orchestrator: Chỉ thực hiện hành động."""
        v, w = action
        
        self.last_action = action
        self.controller.send_command(v*self.max_v, w*self.max_w)
        self.current_timestep += 1

    def calculate_reward_and_done(self, step):
        """
        API cho Orchestrator: Cập nhật state và tính kết quả.
        *** NEW: Sử dụng cờ is_tilted để xác định va chạm. ***
        """
        self.update_state(self.latest_model_states)
        obs_vector = self.lastest_obs
        
        # Lấy trạng thái lật từ state manager
        is_tilted = self.state_manager.is_tilted

        reward, done, goal_reached, crashed_by_reward_logic, min_obstacle_dist = self.reward_calculator.calculate(
            self.state_manager,
            obs_vector,
            self.state_manager.last_nearest_idx,
            step
        )
        
        # --- NEW: Gộp trạng thái lật vào kết quả cuối cùng ---
        # `crashed` sẽ là True nếu logic reward nói vậy HOẶC nếu robot bị lật
        crashed = crashed_by_reward_logic or is_tilted

        # Nếu robot bị lật, ép buộc kết thúc episode
        if is_tilted:
            done = True
            reward -= 400.0 # Phạt cực nặng vì làm lật robot
            rospy.logerr(f"[{self.robot_name}] Episode kết thúc do robot bị lật!")

        # Logic cũ về tiến độ (giữ nguyên)
        if self.state_manager.last_nearest_idx >= self.state_manager.max_progress_idx:
            self.state_manager.update_last_nearest_idx(self.state_manager.last_nearest_idx)
            reward += 5.0

        return reward, done, goal_reached, crashed, min_obstacle_dist

    def reset(self):
        """Reset toàn bộ trạng thái và biến theo dõi của môi trường."""
        self.state_manager.reset()
        self.last_action = np.array([0.0, 0.0])
        self.current_timestep = 0
        #return self._precomputed_obs

    def reset_internal_state(self):
        """Chỉ reset các biến theo dõi cho một episode mới."""
        self.state_manager.reset() # Hàm reset của state_manager chỉ reset index
        self.last_action = np.array([0.0, 0.0])
        self.current_timestep = 0
    
    def set_new_path(self, path_msg):
        """API cho Orchestrator để đặt path mới."""
        if path_msg and len(path_msg.poses) > 1:
            path_points = np.array([(p.pose.position.x, p.pose.position.y) for p in path_msg.poses])
            self.state_manager.set_path(path_points)
        else:
            self.state_manager.set_path(None)

    def close(self):
        """Dọn dẹp tài nguyên."""
        self.controller.stop()
        