# mir_gym_env/include/robot_state.py

import threading
import numpy as np
from tf.transformations import euler_from_quaternion

class RobotState:
    """
    Lưu trữ và quản lý trạng thái của MỘT robot một cách an toàn luồng.
    """
    def __init__(self):
        self.lock = threading.RLock()
        self.pose = (0.0, 0.0, 0.0)  # (x, y, yaw)
        self.velocity = (0.0, 0.0)    # (linear_v, angular_w)
        
        # Trạng thái liên quan đến path
        self.path_points = None         # np.array của các điểm (x, y)
        self.total_path_length = 0.0
        self.last_nearest_idx = 0       # Chỉ số của điểm gần nhất trên path
        self.max_progress_idx = 0       # Chỉ số xa nhất đã đạt được

    def update_kinematics(self, pose_msg, twist_msg):
        """Cập nhật vị trí và vận tốc từ message ROS."""
        with self.lock:
            ori = pose_msg.orientation
            _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
            self.pose = (pose_msg.position.x, pose_msg.position.y, yaw)
            self.velocity = (twist_msg.linear.x, twist_msg.angular.z)

    def set_path(self, path_points):
        """
        Thiết lập một quỹ đạo mới và reset các chỉ số theo dõi.
        """
        with self.lock:
            self.path_points = path_points
            if self.path_points is not None and len(self.path_points) > 1:
                # Tính tổng chiều dài path để chuẩn hóa
                distances = np.linalg.norm(self.path_points[1:] - self.path_points[:-1], axis=1)
                self.total_path_length = np.sum(distances)
            else:
                self.total_path_length = 0.0
            
            # Reset các chỉ số khi có path mới
            self.reset()

    def get_snapshot(self):
        """
        Lấy một bản sao an toàn của tất cả các trạng thái tại một thời điểm.
        """
        with self.lock:
            return (self.pose, self.velocity, self.path_points, 
                    self.last_nearest_idx, self.max_progress_idx, self.total_path_length)

    def update_last_nearest_idx(self, new_idx):
        """Cập nhật một cách an toàn chỉ số điểm gần nhất và tiến độ."""
        with self.lock:
            self.last_nearest_idx = new_idx
            if new_idx > self.max_progress_idx:
                self.max_progress_idx = new_idx
    
    def reset(self):
        """
        Reset các chỉ số theo dõi quỹ đạo cho một episode mới.
        Không reset path, chỉ reset tiến độ.
        """
        with self.lock:
            self.last_nearest_idx = 0
            self.max_progress_idx = 0