import numpy as np
import rospy
import math

class RewardCalculator:
    def __init__(self, robot_name):
        self.robot_name = robot_name
        self.dist_to_path = 0.0
        self.last_nearest_idx = 0

        self.is_stuck = False
        self.stuck_timer = 0
        self.last_min_dist_when_stuck = 0.0
        self.last_dominant_ttc = float('inf')
        # Bán kính robot (ước tính)
        self.robot_radius = 0.5
        # --- CÁC HỆ SỐ CÓ THỂ ĐIỀU CHỈNH ---
        # Tần số cao hơn -> các bước thời gian ngắn hơn -> cần thời gian lâu hơn để kết luận là "kẹt"
        self.STUCK_TIME_LIMIT = 150    # Tương đương ~12 giây ở 12Hz, hoặc ~7.5 giây ở 20Hz
        self.COLLISION_THRESHOLD = 0.6 
        self.ESCAPE_THRESHOLD = 0.7
        # Trọng số cho chế độ hoạt động bình thường
        self.W_FOLLOWING = 1.0
        self.W_AVOIDANCE = 1.0
        self.W_PROGRESS = 1.0
        self.pel_per_step = -0.01
        # Trọng số cho chế độ thoát hiểm (Escape Mode)
        self.W_ESCAPE = 1.0
        self.W_FOLLOWING_IN_ESCAPE = 0.2 # <-- Giữ lại một phần nhỏ
        self.W_PROGRESS_IN_ESCAPE = 0.2  # <-- Giữ lại một phần nhỏ
    def calculate(self, robot_state, obs_vector, current_nearest_idx, step):
        # --- 1. Trích xuất thông tin chung ---
        (current_pose, _, path, _, _, _) = robot_state.get_snapshot()
        if path is None: return -20.0, True, False, False, float('inf')
        
        lidar_info = obs_vector[6:12]
        nearest_robot_rel_pos = obs_vector[22:24]
        min_obstacle_dist = min(
            min(lidar_info[0], lidar_info[2], lidar_info[4]),
            np.linalg.norm(nearest_robot_rel_pos) - 1.0
        )

        state_change_reward = self._update_stuck_state(min_obstacle_dist)
        
        # --- Trích xuất thông tin cho các thành phần reward ---
        ep, alpha, v_signed, w = obs_vector[:4]
        speed = abs(v_signed); alpha_rad = abs(alpha)
        alpha_effective = min(alpha_rad, np.pi - alpha_rad)
        path_approach_speed = obs_vector[4]
        future_points_1d = obs_vector[12:22]
        nearest_robot_rel_vel = obs_vector[24:26]
        dis_to_goal_on_path = obs_vector[26]
        self.dist_to_path = np.linalg.norm(np.array(future_points_1d[0:2]))
        # === TÍNH TOÁN CÁC THÀNH PHẦN REWARD CƠ BẢN (KHÔNG ĐỔI) ===
        reward_following = self._calculate_following_reward(ep, alpha_effective, speed, path_approach_speed, dis_to_goal_on_path)
        avoidance_penalty, avoidance_reward = self._calculate_avoidance_reward_and_penalty(
            v_signed, w, lidar_info, nearest_robot_rel_pos, nearest_robot_rel_vel
        )
        path_curvature = self.estimate_curvature_from_future_points(future_points_1d)
        penalty_turn_speed = -0.5 * (speed**2) * path_curvature if speed > 0.3 else 0.0
        if path_curvature < 0.1:  penalty_turn_speed-= 1.0 * abs(w)                                                                    #phạt quay khi đi thẳng
        action_cost =self.pel_per_step-0.01 * abs(v_signed) - 0.01 * abs(w) # Giảm nhẹ hình phạt hành động
        progress_reward = 1.0 * (current_nearest_idx - self.last_nearest_idx) + 1.5 / (1 + dis_to_goal_on_path)
        self.last_nearest_idx = current_nearest_idx

        # === LOGIC RẼ NHÁNH VÀ TỔNG HỢP REWARD (SẠCH SẼ HƠN) ===
        if self.is_stuck:
            # --- CHẾ ĐỘ THOÁT HIỂM ---
            escape_reward = self._calculate_escape_reward_symmetric(min_obstacle_dist, speed)
            
            total_reward = (
                state_change_reward +
                self.W_ESCAPE * escape_reward +
                self.W_FOLLOWING_IN_ESCAPE * reward_following +
                self.W_PROGRESS_IN_ESCAPE * progress_reward +
                action_cost
            )

            if self.stuck_timer > self.STUCK_TIME_LIMIT:
                total_reward -= 30.0
                self.is_stuck = False
                return total_reward, True, False, True, min_obstacle_dist
            
            return total_reward, False, False, True, min_obstacle_dist

        else:
            # --- CHẾ ĐỘ BÌNH THƯỜNG ---
            total_reward = (
                state_change_reward +
                self.W_FOLLOWING * reward_following +
                self.W_AVOIDANCE * (avoidance_penalty + avoidance_reward) +
                penalty_turn_speed + # Có thể thêm W_TURN_SPEED nếu muốn
                self.W_PROGRESS * progress_reward +
                action_cost
            )

            # ... (Phần logic done và các hình phạt nhỏ không đổi) ...
            path_len = len(path)
            progress_percentage = current_nearest_idx / (path_len - 1) if path_len > 1 else 0
            is_near_end_of_path = progress_percentage >= 0.95
            dist_to_goal = np.linalg.norm(np.array(current_pose[:2]) - path[-1])
            goal_reached = dist_to_goal < 0.4 and is_near_end_of_path
            crashed_status = self.is_stuck
            off_track = abs(self.dist_to_path) > 3.0
            
            if speed < 0.1 and abs(w) > 0.1: total_reward -= 0.1
            elif speed < 0.05: total_reward -= 0.2
            
            done = goal_reached or off_track
            
            if done:
                self.is_stuck = False
                if goal_reached: total_reward += 50.0
                else: total_reward -= 25.0

            return total_reward, done, goal_reached, crashed_status, min_obstacle_dist

    # ===================================================================
    # CÁC HÀM HELPER VỚI HỆ SỐ ĐÃ ĐIỀU CHỈNH
    # ===================================================================

    def _update_stuck_state(self, min_obstacle_dist):
        """Quản lý máy trạng thái: NOT_STUCK <-> STUCK."""
        if self.is_stuck:
            self.stuck_timer += 1
            if min_obstacle_dist > self.ESCAPE_THRESHOLD:
                rospy.loginfo_throttle(1, f"[{self.robot_name}] Đã thoát khỏi va chạm!")
                self.is_stuck = False
                self.stuck_timer = 0
                return 50.0 # GIỮ NGUYÊN - Thưởng sự kiện thoát thành công
        else:
            if min_obstacle_dist < self.COLLISION_THRESHOLD:
                rospy.logwarn_throttle(1, f"[{self.robot_name}] Va chạm! Kích hoạt chế độ thoát hiểm.")
                self.is_stuck = True
                self.stuck_timer = 0
                self.last_min_dist_when_stuck = min_obstacle_dist
                return -150.0 # GIỮ NGUYÊN - Phạt nặng ngay tại thời điểm va chạm
        return 0.0

    def _calculate_escape_reward_symmetric(self, current_min_dist, speed):
        """Tính reward để thoát hiểm. Hệ số đã được điều chỉnh."""
        delta_dist = current_min_dist - self.last_min_dist_when_stuck
        # Tăng nhẹ hệ số để khuyến khích mạnh mẽ việc thoát ra
        escape_reward = 10.0 * delta_dist 
        if speed < 0.1:
            escape_reward -= 0.5 # Giảm từ -5.0
        self.last_min_dist_when_stuck = current_min_dist
        return escape_reward

    def _calculate_following_reward(self, ep, alpha_effective, speed, path_approach_speed, dis_to_goal_on_path):
        """Hệ số đã được điều chỉnh."""
        # Giảm các hệ số hình phạt
        reward_ep = -8.0 * (self.dist_to_path*2) * (1 + abs(speed)) 
        reward_alpha = -10.0 * math.sin(alpha_effective)       
        
        # Giảm hệ số reward tiến độ
        
        reward_vel_approach_goal = 0.0
        if dis_to_goal_on_path < 1.0:
            if speed < 0.2 and math.sin(alpha_effective) < 0.1:
                reward_vel_approach_goal = +0.5 # Giảm từ +5.0
            else:
                reward_vel_approach_goal = -1.0 * speed # Giảm từ -10.0
        else:
            # Thành phần này đã được chuyển ra ngoài
            pass
            
        centering_reward = -0.05 * path_approach_speed # Giảm từ -0.5
        
        return  reward_ep + reward_alpha + centering_reward + reward_vel_approach_goal

    def _calculate_avoidance_reward_and_penalty(self, current_v, current_w, lidar_info, nearest_robot_rel_pos, nearest_robot_rel_vel):
        """
        Logic được thiết kế lại để xử lý chính xác cả nguy hiểm do tiến/lùi (v) và quay (w).
        """
        # --- 1. Tính toán nguy hiểm từ vật cản động (không đổi) ---
        dist_sq_dyn = np.dot(nearest_robot_rel_pos, nearest_robot_rel_pos)
        dist_dyn = np.sqrt(dist_sq_dyn) if dist_sq_dyn > 1e-9 else 0.1
        approach_speed_dyn = -np.dot(nearest_robot_rel_pos, nearest_robot_rel_vel) / (dist_dyn + 1e-6)
        ttc_dyn = dist_dyn / (approach_speed_dyn + 1e-6) if approach_speed_dyn > 0 else float('inf')

        # --- 2. Tính toán nguy hiểm từ vật cản tĩnh ---
        min_dist_static = float('inf')
        angle_of_min_dist = 0
        # Giả định lidar_info là [dist_0, angle_0, dist_1, angle_1, ...]
        for i in range(0, len(lidar_info), 2):
            dist = lidar_info[i]
            if dist < min_dist_static:
                min_dist_static = dist
                angle_of_min_dist = lidar_info[i+1]
        
        # --- 2a. Nguy hiểm do tiếp cận thẳng (do v) ---
        approach_speed_radial = current_v * math.cos(angle_of_min_dist)
        ttc_radial = min_dist_static / (approach_speed_radial + 1e-6) if approach_speed_radial > 0 else float('inf')

        # --- 2b. Nguy hiểm do quét ngang (do w) ---
        
        # Vận tốc quét ngang lớn nhất (tại hông robot, vuông góc với hướng tiến)
        lateral_speed = abs(current_w) * self.robot_radius
        # Khoảng cách an toàn ngang. Robot cần ít nhất 'robot_radius' không gian trống để quay.
        # Ta có thể dùng min_dist_static để ước tính khoảng trống này.
        # Time-to-impact = (khoảng cách an toàn - bán kính) / vận tốc quét
        # Nếu min_dist_static < robot_radius, có nghĩa là đã có khả năng va chạm
        clearance = min_dist_static - self.robot_radius
        ttc_lateral = clearance / (lateral_speed + 1e-6) if lateral_speed > 0 and clearance > 0 else 0.0
        
        # TTC của vật cản tĩnh là trường hợp tệ nhất giữa tiếp cận thẳng và quét ngang
        ttc_static = min(ttc_radial, ttc_lateral)

        # --- 3. Xác định mối nguy hiểm chính (dominant threat) ---
        if ttc_dyn < ttc_static:
            dominant_threat_ttc, dominant_threat_dist = ttc_dyn, dist_dyn
        else:
            dominant_threat_ttc, dominant_threat_dist = ttc_static, min_dist_static
                
        # --- 4. Áp dụng hình phạt ---
        # Sử dụng các ngưỡng đã được điều chỉnh cho không gian hẹp
        TTC_threshold = 2.5
        safety_dist = 0.7
        penalty = 0.0
        reward = 0.0
        
        if dominant_threat_ttc < TTC_threshold or dominant_threat_dist < safety_dist:
            rospy.logwarn_throttle(1, f"[{self.robot_name}] Nguy hiểm! TTC: {dominant_threat_ttc:.2f}s, Dist: {dominant_threat_dist:.2f}m")
            
            # 1. Tính toán sự thay đổi của TTC
            # Nếu TTC trước đó cũng nguy hiểm, hãy xem xét sự thay đổi
            if self.last_dominant_ttc < TTC_threshold:
                delta_ttc = dominant_threat_ttc - self.last_dominant_ttc
                
                # Nếu delta_ttc > 0, có nghĩa là TTC đang tăng -> hành động tốt!
                if delta_ttc > 0:
                    # Thưởng cho việc làm tăng TTC (trở nên an toàn hơn)
                    # Hệ số 10.0 để tạo ra một phần thưởng đáng kể
                    reward += 10.0 * delta_ttc
            
            # 2. Tính toán hình phạt như cũ
            # Phạt dựa trên mức độ nguy hiểm hiện tại (TTC thấp)
            penalty = -5.0 * np.exp(-dominant_threat_ttc / 2.0)
            
            current_speed_abs = abs(current_v)
            if current_speed_abs > 0.3 and dominant_threat_dist < safety_dist * 1.5:
                penalty -= (current_speed_abs ** 2) * 2.0
        
        # Cập nhật last_ttc cho bước tiếp theo
        self.last_dominant_ttc = dominant_threat_ttc
                    
        return penalty, reward

    def estimate_curvature_from_future_points(self, future_points_1d_list):
        # (Logic này không đổi)
        if len(future_points_1d_list) < 6: return 0.0
        p1 = np.array(future_points_1d_list[0:2]); p2 = np.array(future_points_1d_list[2:4]); p3 = np.array(future_points_1d_list[4:6])
        if np.allclose(p1, p2) or np.allclose(p2, p3) or np.allclose(p1, p3): return 0.0
        a = np.linalg.norm(p2-p1); b = np.linalg.norm(p3-p2); c = np.linalg.norm(p3-p1)
        if a * b * c < 1e-9: return 0.0
        s = (a+b+c)/2.0; area_sq = s * max(0, s-a) * max(0, s-b) * max(0, s-c)
        if area_sq <= 0: return 0.0
        area = np.sqrt(area_sq); curvature = (4*area)/(a*b*c)
        return np.clip(curvature, 0, 4.0)