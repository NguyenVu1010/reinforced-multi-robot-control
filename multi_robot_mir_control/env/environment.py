#!/usr/bin/env python3

# src/mir_gym_env/environment.py

import rospy
import gym
from gym import spaces
import numpy as np

# Import các thành phần logic (giả sử chúng nằm trong thư mục con)
from .include.robot_state import RobotState
from .include.robot_controller import RobotController
from .include.observation_calculator import ObservationCalculator
from .include.reward_calculator import RewardCalculator

# Import các kiểu message ROS
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan

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
        self.obs_calculator = ObservationCalculator(robot_name)
        self.reward_calculator = RewardCalculator(robot_name)
        
        # --- SPACES ---
        self.action_space = spaces.Box(low=np.array([-1.0, -0.8]), high=np.array([1.0, 0.8]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(self.cfg['state_dim'],), dtype=np.float32)

        # --- BIẾN TRẠNG THÁI ---
        self.last_action = np.array([0.0, 0.0])
        self.other_robots_states_cache = {}
        self.latest_lidar_data = None
        
        # Biến để theo dõi timestep
        self.current_timestep = 0
        self.max_steps = self.cfg['max_steps_per_episode']
        
        # --- ROS ---
        # Chỉ cần 1 subscriber cho robot này
        lidar_topic = f'/{self.robot_name}/scan'
        self.lidar_sub = rospy.Subscriber(lidar_topic, LaserScan, self._lidar_callback, queue_size=1)
        self.rate = rospy.Rate(20)

    def _lidar_callback(self, msg):
        """Lưu lại dữ liệu Lidar mới nhất cho robot này."""
        self.latest_lidar_data = msg

    def update_state(self, model_states_msg):
        """
        Cập nhật trạng thái của robot này và cache trạng thái của các robot khác
        từ message /gazebo/model_states chung.
        """
        try:
            # Cập nhật cho chính mình
            my_idx = model_states_msg.name.index(self.robot_name)
            self.state_manager.update_kinematics(model_states_msg.pose[my_idx], model_states_msg.twist[my_idx])

            # Cache trạng thái của các robot khác
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
            # Có thể xảy ra khi message đến không đầy đủ, bỏ qua bước cập nhật này
            pass

    def _get_obs(self):
        """
        Tính toán và trả về observation cho robot này.
        """
        normalized_timestep = self.current_timestep / self.max_steps
        
        return self.obs_calculator.calculate(
            robot_state_obj=self.state_manager,
            other_robots_state=self.other_robots_states_cache,
            lidar_data=self.latest_lidar_data,
            current_timestep_normalized=normalized_timestep
        )

    def update_state_and_get_obs(self, model_states_msg):
        """API cho Orchestrator: Gộp 2 bước cập nhật và lấy obs."""
        self.update_state(model_states_msg)
        return self._get_obs()

    def step(self, action,alpha):
        """API cho Orchestrator: Chỉ thực hiện hành động."""
        v, w = action
        # if abs(alpha) > 1.3:
        #     v = 0.0  # không cho tiến khi lệch hướng lớn
        self.last_action = action
        self.controller.send_command(v, w)
        self.current_timestep += 1

    def calculate_reward_and_done(self, model_states_msg,step):
        """API cho Orchestrator: Cập nhật state và tính kết quả."""
        self.update_state(model_states_msg)
        obs_vector = self._get_obs() # Cần obs mới nhất để tính reward
        
        reward, done, goal_reached,crashed, min_obstacle_dist = self.reward_calculator.calculate(
            self.state_manager,
            self.other_robots_states_cache,
            self.last_action,
            obs_vector,
            self.state_manager.last_nearest_idx,
            step
            )
        if self.state_manager.last_nearest_idx >= self.state_manager.max_progress_idx:
            self.state_manager.update_last_nearest_idx(self.state_manager.last_nearest_idx)
            reward += 5.0

        return reward, done, goal_reached, crashed, min_obstacle_dist

    def reset(self):
        """Reset toàn bộ trạng thái và biến theo dõi của môi trường."""
        self.state_manager.reset()
        self.last_action = np.array([0.0, 0.0])
        self.current_timestep = 0
        return self._get_obs()

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
        if self.lidar_sub:
            self.lidar_sub.unregister()