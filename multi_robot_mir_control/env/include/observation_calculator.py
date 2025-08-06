# mir_gym_env/include/observation_calculator.py

import rospy
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from sensor_msgs.msg import LaserScan
from scipy.signal import find_peaks
from visualization_msgs.msg import Marker

class ObservationCalculator:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.num_lidar_clusters = 3
        # Bạn có thể thêm các debug publisher ở đây nếu muốn
        debug_topic = f'/debug/{robot_name}/future_points_marker'
        self.points_marker_pub = rospy.Publisher(debug_topic, Marker, queue_size=1)

    def calculate(self, robot_state_obj, other_robots_state, lidar_data, current_timestep_normalized):
        """
        Tính toán vector quan sát 27 chiều.
        """
        # Lấy trạng thái từ snapshot
        (current_pose, current_velocity, path, 
         last_idx, max_progress_idx, total_path_len) = robot_state_obj.get_snapshot()

        # 1. Path Following (2) + Future Points (10)
        ep, alpha, _, future_points_10d = self._get_path_properties(
            current_pose, current_velocity, path, last_idx, robot_state_obj, num_future_points=5
        )

        # 2. Robot Kinematics (2)
        v, w = current_velocity
        
        # 3. Path Approach/Departure Speed (1)
        path_approach_speed = self._calculate_path_approach_speed(
            current_pose, current_velocity, path, last_idx
        )

        # 4. Normalized Timestep (1) - đã được truyền vào

        # 5. Lidar Obstacles (6)
        obstacle_info = self._get_lidar_obstacle_info(lidar_data)

        # 6. Nearest Robot Full Info (4)
        rel_pos, rel_vel = self._get_nearest_robot_full_info(
            current_pose, current_velocity, other_robots_state
        )

        # 7. Remaining Path Distance (1)
        dist_to_goal_on_path = self._calculate_remaining_path_distance(
            path, max_progress_idx, total_path_len
        )

        # Gộp thành vector 27 chiều
        observation_vector = np.concatenate([
            np.array([ep, alpha]),
            np.array([v, w]),
            np.array([path_approach_speed]),
            np.array([current_timestep_normalized]),
            obstacle_info.flatten(),
            np.array(future_points_10d),
            rel_pos,
            rel_vel,
            np.array([dist_to_goal_on_path])
        ])
        
        return observation_vector.astype(np.float32)

    # --- CÁC HÀM HELPER ---

    def _get_path_properties(self, current_pose, current_velocity, path, last_idx, robot_state_obj, 
                             num_future_points=5, search_window=400):
        if path is None or len(path) < 2:
            return 0, 0, True, [0.0] * (num_future_points * 2)

        robot_pos = np.array(current_pose[:2])
        robot_yaw = current_pose[2]
        linear_v = current_velocity[0]
        
        # --- Phần 1: Tìm điểm gần nhất (giữ nguyên) ---
        start_search = max(0, last_idx - search_window // 2)
        end_search = min(len(path), last_idx + search_window // 2)
        path_slice = path[start_search:end_search]
        if len(path_slice) == 0:
            distances = np.linalg.norm(path - robot_pos, axis=1)
            nearest_idx = np.argmin(distances)
        else:
            distances_slice = np.linalg.norm(path_slice - robot_pos, axis=1)
            nearest_idx = start_search + np.argmin(distances_slice)
        robot_state_obj.update_last_nearest_idx(nearest_idx)

        # --- Phần 2: Tính ep và alpha (giữ nguyên) ---
        if nearest_idx < len(path) - 1:
            p1, p2 = path[nearest_idx], path[nearest_idx + 1]
        else:
            p1, p2 = path[nearest_idx - 1], path[nearest_idx]
        
        vec_path_to_robot = robot_pos - p1
        path_tangent = p2 - p1
        path_normal = np.array([-path_tangent[1], path_tangent[0]])
        ep = -np.dot(vec_path_to_robot, path_normal) / (np.linalg.norm(path_normal) + 1e-6)

        look_ahead_dist = np.clip(linear_v * 1.0, 0.4, 1.2)
        target_idx_for_alpha = nearest_idx
        while target_idx_for_alpha < len(path) - 1 and np.linalg.norm(path[target_idx_for_alpha] - path[nearest_idx]) < look_ahead_dist:
            target_idx_for_alpha += 1
        
        angle_to_target = np.arctan2(path[target_idx_for_alpha][1] - robot_pos[1], path[target_idx_for_alpha][0] - robot_pos[0])
        alpha = (angle_to_target - robot_yaw + np.pi) % (2 * np.pi) - np.pi

        # --- Phần 3: TÍNH TOÁN FUTURE POINTS (LOGIC MỚI, AN TOÀN) ---
        future_points_robot_frame_pairs = []
        future_points_world_frame_pairs = []
        
        future_dist_step = 0.5
        cos_yaw, sin_yaw = np.cos(-robot_yaw), np.sin(-robot_yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        
        for i in range(num_future_points):
            # Khoảng cách mục tiêu cho điểm thứ i, tính từ điểm gần nhất
            target_dist = (i + 1) * future_dist_step
            
            # Bắt đầu tìm kiếm lại từ điểm gần nhất cho mỗi future point
            temp_idx = nearest_idx
            dist_traveled = 0.0
            
            # Duyệt về phía trước trên path
            while temp_idx < len(path) - 1 and dist_traveled < target_dist:
                dist_traveled += np.linalg.norm(path[temp_idx+1] - path[temp_idx])
                temp_idx += 1
            
            # Lấy điểm trên path (kẹp ở cuối để tránh lỗi index)
            point_world = path[min(temp_idx, len(path)-1)]
            future_points_world_frame_pairs.append(point_world)
            
            # Chuyển về hệ robot
            vec_to_future = point_world - robot_pos
            point_robot = rotation_matrix @ vec_to_future
            future_points_robot_frame_pairs.append(point_robot)

        # --- Phần 4: Publish và Return ---
        self._publish_future_points_marker(future_points_world_frame_pairs, self.robot_name)
        
        goal_reached = np.linalg.norm(robot_pos - path[-1]) < 0.3
        
        flat_future_points_robot = np.array(future_points_robot_frame_pairs).flatten().tolist()
        
        return ep, alpha, goal_reached, flat_future_points_robot

    def _calculate_path_approach_speed(self, current_pose, current_velocity, path, nearest_idx):
        if path is None or len(path) < 2 or nearest_idx >= len(path): return 0.0
        robot_pos = np.array(current_pose[:2])
        vec_path_to_robot = robot_pos - path[nearest_idx]
        if np.linalg.norm(vec_path_to_robot) < 1e-4: return 0.0
        linear_v, robot_yaw = current_velocity[0], current_pose[2]
        robot_vel_vec = np.array([linear_v * np.cos(robot_yaw), linear_v * np.sin(robot_yaw)])
        approach_speed = np.dot(robot_vel_vec, vec_path_to_robot) / np.linalg.norm(vec_path_to_robot)
        return np.tanh(approach_speed)

    import numpy as np

    def _get_lidar_obstacle_info(self, lidar_data, max_dist=15.0, min_valid_dist=0.15):
        if lidar_data is None or not lidar_data.ranges:
            return np.array([max_dist, 0.0] * self.num_lidar_clusters)

        # Lấy dữ liệu khoảng cách
        ranges = np.array(lidar_data.ranges)
        ranges[np.isinf(ranges) | np.isnan(ranges)] = max_dist

        # Tính các góc tương ứng với mỗi tia
        num_rays = len(ranges)
        angles = lidar_data.angle_min + np.arange(num_rays) * lidar_data.angle_increment

        # Chia đều thành num_lidar_clusters cụm
        cluster_size = num_rays // self.num_lidar_clusters
        obstacle_info = []

        for i in range(self.num_lidar_clusters):
            start_idx = i * cluster_size
            end_idx = (i + 1) * cluster_size if i < self.num_lidar_clusters - 1 else num_rays

            cluster_ranges = ranges[start_idx:end_idx]
            cluster_angles = angles[start_idx:end_idx]

            # Lọc ra những điểm hợp lệ (>= min_valid_dist)
            valid_mask = cluster_ranges >= min_valid_dist
            valid_ranges = cluster_ranges[valid_mask]
            valid_angles = cluster_angles[valid_mask]

            if len(valid_ranges) == 0:
                obstacle_info.extend([max_dist, 0.0])
            else:
                min_idx = np.argmin(valid_ranges)
                min_range = valid_ranges[min_idx]
                min_angle = valid_angles[min_idx]
                obstacle_info.extend([min_range, min_angle])

        return np.array(obstacle_info)

    def _get_nearest_robot_full_info(self, my_pose, my_velocity, other_states):
        relative_pos_robot = np.array([10.0, 10.0]) 
        relative_vel_robot_frame = np.array([0.0, 0.0])
        if other_states:
            my_pos_vec = np.array(my_pose[:2])
            nearest_other_name = min(other_states, key=lambda k: np.linalg.norm(my_pos_vec - other_states[k]['pos']), default=None)
            if nearest_other_name:
                nearest_other = other_states[nearest_other_name]
                relative_pos_world = nearest_other['pos'] - my_pos_vec
                cos_yaw, sin_yaw = np.cos(-my_pose[2]), np.sin(-my_pose[2])
                rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
                relative_pos_robot = np.clip(rotation_matrix @ relative_pos_world, -10.0, 10.0)
                linear_v, _ = my_velocity
                my_vel_vec = np.array([linear_v * np.cos(my_pose[2]), linear_v * np.sin(my_pose[2])])
                relative_vel_world = nearest_other['vel'] - my_vel_vec
                relative_vel_robot_frame = rotation_matrix @ relative_vel_world
        return relative_pos_robot, relative_vel_robot_frame

    def _calculate_remaining_path_distance(self, path, current_progress_idx, total_path_length):
        if path is None or current_progress_idx >= len(path) - 1 or total_path_length == 0:
            return 0.0
        remaining_dist = sum(np.linalg.norm(path[i+1] - path[i]) for i in range(current_progress_idx, len(path) - 1))
        return np.clip(remaining_dist / total_path_length, 0.0, 1.0)
    def _publish_future_points_marker(self, points_in_world_frame, robot_name):
        """
        [HÀM MỚI]
        Publish tất cả các điểm nhìn trước dưới dạng một Marker duy nhất.
        
        Args:
            points_in_world_frame (list of np.ndarray): Danh sách các điểm [x, y].
            robot_name (str): Tên của robot để đặt namespace cho marker.
        """
        if not points_in_world_frame:
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        
        # Mỗi marker cần một namespace và id duy nhất để không ghi đè lên nhau
        marker.ns = f"future_points_{robot_name}"
        marker.id = 0
        
        # Loại marker là POINTS, hiển thị các điểm riêng lẻ
        marker.type = Marker.POINTS
        
        # Action là ADD/MODIFY
        marker.action = Marker.ADD
        
        # Kích thước của các điểm
        marker.scale.x = 0.1  # đường kính 10cm
        marker.scale.y = 0.1
        marker.scale.z = 0.1 # không quan trọng với POINTS
        
        # Màu sắc của các điểm (ví dụ: màu xanh lá cây)
        marker.color.g = 1.0
        marker.color.a = 1.0 # Alpha = 1.0 (không trong suốt)
        
        # Vòng đời của marker (0 nghĩa là tồn tại mãi mãi cho đến khi bị xóa/cập nhật)
        marker.lifetime = rospy.Duration(0.5) # Tự biến mất sau 0.5s nếu không được cập nhật

        # Điền vào danh sách các điểm
        for point_xy in points_in_world_frame:
            p = Point()
            p.x = point_xy[0]
            p.y = point_xy[1]
            p.z = 0.2 # Nâng lên một chút để dễ nhìn
            marker.points.append(p)
            
        # Publish marker
        self.points_marker_pub.publish(marker)