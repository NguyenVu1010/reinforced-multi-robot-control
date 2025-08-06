import numpy as np
import rospy
import math

class RewardCalculator:
    def __init__(self, robot_name):
        # --- CÁC BIẾN GỐC CỦA BẠN ---
        self.robot_name = robot_name
        self.dist_to_path = 0.0
        self.last_nearest_idx = 0
        # self.count_crashed = 0 # Biến này sẽ được thay thế bằng hệ thống mới

        # --- CÁC BIẾN MỚI CHO CHẾ ĐỘ THOÁT HIỂM ---
        self.is_stuck = False
        self.stuck_timer = 0
        self.last_min_dist_when_stuck = 0.0

        # Các hằng số để điều chỉnh hành vi thoát hiểm
        self.STUCK_TIME_LIMIT = 15     # Số bước tối đa cho phép bị kẹt
        self.COLLISION_THRESHOLD = 0.7 # Ngưỡng khoảng cách để coi là va chạm/bị kẹt
        self.ESCAPE_THRESHOLD = 1.0    # Ngưỡng khoảng cách an toàn để coi là đã thoát

    def calculate(self, robot_state, other_robots_state, last_action, obs_vector, current_nearest_idx, step):
        """
        Tính toán reward. Nếu bị kẹt, sẽ chuyển sang chế độ thoát hiểm.
        Nếu không, sẽ sử dụng logic tính reward gốc của bạn.
        """
        # --- 1. Trích xuất thông tin & Tính khoảng cách nguy hiểm (chung cho cả 2 chế độ) ---
        (current_pose, _, path, _, _, _) = robot_state.get_snapshot()
        if path is None:
            return -200.0, True, False, False, float('inf')

        lidar_info = obs_vector[6:12]
        nearest_robot_rel_pos = obs_vector[22:24]
        dist_to_lidar_obs = min(lidar_info[0], lidar_info[2], lidar_info[4])
        dist_to_dyn_obs = np.linalg.norm(nearest_robot_rel_pos) - 1.0
        min_obstacle_dist = min(dist_to_lidar_obs, dist_to_dyn_obs)

        # --- 2. Cập nhật trạng thái Bị kẹt/Thoát hiểm ---
        state_change_reward = self._update_stuck_state(min_obstacle_dist)

        # --- 3. LOGIC RẼ NHÁNH: THOÁT HIỂM hay BÌNH THƯỜNG? ---

        # ===>>> NẾU ĐANG BỊ KẸT (is_stuck == True), KÍCH HOẠT CHẾ ĐỘ THOÁT HIỂM
        if self.is_stuck:
            v_signed = obs_vector[2]
            speed = abs(v_signed)
            
            escape_reward = self._calculate_escape_reward_symmetric(min_obstacle_dist, speed)
            total_reward = escape_reward + state_change_reward

            # Kiểm tra xem có bị kẹt quá lâu không
            if self.stuck_timer > self.STUCK_TIME_LIMIT:
                rospy.logwarn(f"[{self.robot_name}] Thất bại! Không thể thoát khỏi va chạm.")
                total_reward -= 300.0  # Phạt nặng vì không thoát được
                self.is_stuck = False  # Reset trạng thái cho episode sau
                # Trả về: reward, done=True, goal=False, crashed=True, dist
                return total_reward, True, False, True, min_obstacle_dist
            
            # Nếu chưa hết giờ, tiếp tục cố gắng thoát
            # Trả về: reward, done=False, goal=False, crashed=True, dist
            return total_reward, False, False, True, min_obstacle_dist

        # ===>>> NẾU KHÔNG BỊ KẸT (is_stuck == False), DÙNG TOÀN BỘ LOGIC GỐC CỦA BẠN
        else:
            # --- 1. Trích xuất thông tin (phần còn lại từ code gốc) ---
            ep, alpha, v_signed, w = obs_vector[:4]
            speed = abs(v_signed)
            alpha_rad = abs(alpha)
            alpha_effective = min(alpha_rad, np.pi - alpha_rad)
            path_approach_speed = obs_vector[4]
            future_points_1d = obs_vector[12:22]
            nearest_robot_rel_vel = obs_vector[24:26]
            dis_to_goal_on_path = obs_vector[26]
            
            self.dist_to_path = np.linalg.norm(np.array(future_points_1d[0:2]))

            # --- 2. Tính toán các thành phần Reward/Penalty (CODE GỐC CỦA BẠN) ---
            reward_following = self._calculate_following_reward(ep, alpha_effective, speed, path_approach_speed, dis_to_goal_on_path)
            avoidance_penalty, avoidance_reward = self._calculate_avoidance_reward_and_penalty(
                speed, lidar_info, nearest_robot_rel_pos, nearest_robot_rel_vel
            )
            path_curvature = self.estimate_curvature_from_future_points(future_points_1d)
            penalty_turn_speed = -5.0 * (speed**2) * path_curvature if speed > 0.3 else 0.0
            action_cost = -0.05 * step
            
            progress_reward = 10 * (current_nearest_idx - self.last_nearest_idx) + 15.0 / (1 + dis_to_goal_on_path)
            self.last_nearest_idx = current_nearest_idx
            
            # --- 3. Tổng hợp Reward (CODE GỐC CỦA BẠN) ---
            total_reward = (
                state_change_reward + # Thưởng 50.0 nếu vừa thoát thành công
                reward_following +
                avoidance_penalty +
                avoidance_reward +
                penalty_turn_speed +
                action_cost +
                progress_reward
            )

            # --- 4. Điều kiện kết thúc (Done) (LOGIC GỐC ĐƯỢC ĐIỀU CHỈNH) ---
            path_len = len(path)
            progress_percentage = current_nearest_idx / (path_len - 1) if path_len > 1 else 0
            is_near_end_of_path = progress_percentage >= 0.95
            dist_to_goal = np.linalg.norm(np.array(current_pose[:2]) - path[-1])
            goal_reached = dist_to_goal < 0.4 and is_near_end_of_path
            
            # Logic `crashed_remain` cũ được thay thế bằng `is_stuck`
            # Không cần `count_crashed` nữa
            crashed_status = self.is_stuck 
            off_track = abs(self.dist_to_path) > 3.0
            
            # Phạt các hành vi không hiệu quả (CODE GỐC CỦA BẠN)
            if speed < 0.1 and abs(w) > 0.1:
                total_reward -= 1.0
            elif speed < 0.05:
                total_reward -= 2.0
            
            done = goal_reached or off_track
            
            # Logic phạt kết thúc episode (CODE GỐC CỦA BẠN)
            if done:
                self.is_stuck = False # Reset khi kết thúc
                if goal_reached:
                    total_reward += 500.0
                else: # off_track
                    total_reward -= 250.0

            return total_reward, done, goal_reached, crashed_status, min_obstacle_dist

    # ===================================================================
    # CÁC HÀM HELPER MỚI ĐỂ THOÁT HIỂM
    # ===================================================================

    def _update_stuck_state(self, min_obstacle_dist):
        """Quản lý máy trạng thái: NOT_STUCK <-> STUCK."""
        if self.is_stuck:
            self.stuck_timer += 1
            if min_obstacle_dist > self.ESCAPE_THRESHOLD:
                rospy.loginfo_throttle(1, f"[{self.robot_name}] Đã thoát khỏi va chạm!")
                self.is_stuck = False
                self.stuck_timer = 0
                return 50.0 # Thưởng vì thoát thành công
        else:
            if min_obstacle_dist < self.COLLISION_THRESHOLD:
                rospy.logwarn_throttle(1, f"[{self.robot_name}] Va chạm! Kích hoạt chế độ thoát hiểm.")
                self.is_stuck = True
                self.stuck_timer = 0
                self.last_min_dist_when_stuck = min_obstacle_dist
                return -150.0 # Phạt nặng ngay tại thời điểm va chạm
        return 0.0

    def _calculate_escape_reward_symmetric(self, current_min_dist, speed):
        """Tính reward để thoát hiểm: thưởng cho việc tăng khoảng cách tới vật cản."""
        delta_dist = current_min_dist - self.last_min_dist_when_stuck
        escape_reward = 150.0 * delta_dist
        if speed < 0.1:
            escape_reward -= 5.0
        self.last_min_dist_when_stuck = current_min_dist
        return escape_reward

    # ===================================================================
    # CÁC HÀM HELPER GỐC CỦA BẠN (KHÔNG THAY ĐỔI)
    # ===================================================================

    def _calculate_following_reward(self, ep, alpha_effective, speed, path_approach_speed, dis_to_goal_on_path):
        reward_ep = -50.0 * (self.dist_to_path**2) * (1 + speed)
        reward_alpha = -10.0 * math.sin(alpha_effective)
        progress_reward = 10.0 * speed * math.cos(alpha_effective)
        reward_vel_approach_goal = 0.0
        if dis_to_goal_on_path < 0.5:
            if speed < 0.2 and math.sin(alpha_effective) < 0.1:
                reward_vel_approach_goal = +5.0
            else:
                reward_vel_approach_goal = -10.0 * speed
        else:
            progress_reward += 15.0 / (1 + dis_to_goal_on_path)
        centering_reward = -0.5 * path_approach_speed
        return progress_reward + reward_ep + reward_alpha + centering_reward + reward_vel_approach_goal

    def _calculate_avoidance_reward_and_penalty(self, current_speed, lidar_info, nearest_robot_rel_pos, nearest_robot_rel_vel):
        dist_sq_dyn = np.dot(nearest_robot_rel_pos, nearest_robot_rel_pos)
        approach_speed_dyn = np.dot(nearest_robot_rel_pos, nearest_robot_rel_vel)
        ttc_dyn = float('inf')
        if approach_speed_dyn < 0 and dist_sq_dyn > 1e-6:
            ttc_dyn = -dist_sq_dyn / (approach_speed_dyn - 1e-6)
        min_dist_static = float('inf')
        for i in range(0, len(lidar_info), 2):
            dist = lidar_info[i]
            if dist < min_dist_static:
                min_dist_static = dist
        ttc_static = min_dist_static / (current_speed + 1e-6) if current_speed > 0 else float('inf')
        if ttc_dyn < ttc_static:
            dominant_threat_ttc = ttc_dyn
            dominant_threat_dist = np.sqrt(dist_sq_dyn)
        else:
            dominant_threat_ttc = ttc_static
            dominant_threat_dist = min_dist_static
        TTC_threshold = 5.0
        safety_dist = 2.0
        penalty = 0.0
        reward = 0.0
        if dominant_threat_ttc < TTC_threshold or dominant_threat_dist < safety_dist:
            rospy.logwarn_throttle(1, f"[{self.robot_name}] Nguy hiểm! TTC: {dominant_threat_ttc:.2f}s, Dist: {dominant_threat_dist:.2f}m")
            penalty = -50.0 * np.exp(-dominant_threat_ttc / 2.0)
            if current_speed > 0.3:
                penalty -= (current_speed ** 2) * 20.0
        return penalty, reward

    def estimate_curvature_from_future_points(self, future_points_1d_list):
        if len(future_points_1d_list) < 6: return 0.0
        p1 = np.array(future_points_1d_list[0:2])
        p2 = np.array(future_points_1d_list[2:4])
        p3 = np.array(future_points_1d_list[4:6])
        if np.allclose(p1, p2) or np.allclose(p2, p3) or np.allclose(p1, p3): return 0.0
        a = np.linalg.norm(p2 - p1); b = np.linalg.norm(p3 - p2); c = np.linalg.norm(p3 - p1)
        if a * b * c < 1e-9: return 0.0
        s = (a + b + c) / 2.0
        area_sq = s * max(0, s - a) * max(0, s - b) * max(0, s - c)
        if area_sq <= 0: return 0.0
        area = np.sqrt(area_sq)
        curvature = (4 * area) / (a * b * c)
        return np.clip(curvature, 0, 4.0)