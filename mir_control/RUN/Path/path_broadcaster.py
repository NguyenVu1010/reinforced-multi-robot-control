#!/usr/bin/env python3

import rospy
import numpy as np
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Path
# Import service đã được định nghĩa và biên dịch
from mir_control.srv import ProvidePath, ProvidePathResponse 

class PathProviderServer:
    """
    Hoạt động như một service server. Khi nhận được yêu cầu, nó tạo ra một
    quỹ đạo ngẫu nhiên, dài, gấp khúc và hướng về gốc, sau đó trả về
    quỹ đạo đó trực tiếp cho client.
    """
    def __init__(self):
        self.robot_poses = {}
        # Subscriber để luôn biết vị trí của tất cả robot
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.path_publishers = {}
        # Tạo Service Server
        self.service = rospy.Service('provide_path', ProvidePath, self.handle_provide_path_request)
        rospy.loginfo("Path Provider Service đã sẵn sàng và chờ yêu cầu.")
    def get_publisher_for_robot(self, robot_name):
        """Tạo hoặc lấy lại publisher cho một robot cụ thể."""
        if robot_name not in self.path_publishers:
            # Sửa tên topic để rõ ràng hơn, ví dụ /paths/robot1
            topic_name = f'/paths/{robot_name}/planned_path'
            self.path_publishers[robot_name] = rospy.Publisher(topic_name, Path, queue_size=1, latch=True)
            #rospy.loginfo(f"Đã tạo publisher cho '{robot_name}' trên topic '{topic_name}'")
        return self.path_publishers[robot_name]
    def model_states_callback(self, msg):
        """Cập nhật vị trí của tất cả các model robot."""
        for i, name in enumerate(msg.name):
            if 'robot' in name:
                pose_msg = msg.pose[i]
                ori = pose_msg.orientation
                _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
                self.robot_poses[name] = (pose_msg.position.x, pose_msg.position.y, yaw)

    def generate_final_path(self, start_pos, num_waypoints=5,
                        target_pos=np.array([0.0, 0.0]), bias_weight=0.3,
                        curvature_prob=0.5, segment_length_range=(6.0, 12.0),
                        step_size=0.02):
        """
        Sinh path gồm một chuỗi waypoint hướng về target_pos, rồi nội suy đều các điểm
        giữa các waypoint sao cho khoảng cách giữa hai điểm liên tiếp ≈ step_size.
        """
        # 1. Sinh các waypoint rời rạc
        waypoints = [np.array(start_pos)]
        current_pos = np.array(start_pos)
        for _ in range(num_waypoints - 1):
            # hướng về target + nhiễu
            dir_to_goal = target_pos - current_pos
            norm = np.linalg.norm(dir_to_goal)
            dir_to_goal = dir_to_goal / (norm + 1e-6) if norm > 1e-6 else np.zeros(2)
            noise = np.random.uniform(-1, 1, size=2)
            direction = (1 - bias_weight) * noise + bias_weight * dir_to_goal
            direction_ = direction / (np.linalg.norm(direction) + 1e-6)
            # độ dài đoạn ngẫu nhiên trong khoảng
            seg_len = np.random.uniform(*segment_length_range)
            next_pos = current_pos + direction_ * seg_len
            waypoints.append(next_pos)
            current_pos = next_pos

        # 2. Nội suy đều các đoạn
        path_list_dense = []
        for i in range(len(waypoints) - 1):
            p_start = waypoints[i]
            p_end = waypoints[i + 1]

            if np.random.rand() < curvature_prob and i <= len(waypoints) - 2:
                # tạo cung cong
                offset_dir = np.array([-(p_end - p_start)[1], (p_end - p_start)[0]])  # vuông góc
                offset_dir = offset_dir / (np.linalg.norm(offset_dir) + 1e-6)
                curvature_strength = np.random.uniform(1.0, 3.0)
                p_mid = (p_start + p_end) / 2 + offset_dir * curvature_strength

                arc_pts = self.create_arc(p_start, p_mid, p_end, step_size=step_size)

                if i > 0:
                    # tránh lặp lại điểm đầu trùng đoạn trước
                    arc_pts = arc_pts[1:]

                path_list_dense.extend(arc_pts)

            else:
                # nội suy thẳng đều
                seg_vec = p_end - p_start
                seg_len = np.linalg.norm(seg_vec)
                unit_vec = seg_vec / (seg_len + 1e-6)
                segment_pts = []

                cur_pt = p_start.copy()
                segment_pts.append(cur_pt.tolist())

                while True:
                    remaining = np.linalg.norm(p_end - cur_pt)
                    if remaining <= step_size:
                        segment_pts.append(p_end.tolist())
                        break
                    else:
                        cur_pt = cur_pt + unit_vec * step_size
                        segment_pts.append(cur_pt.tolist())

                if i > 0:
                    segment_pts = segment_pts[1:]  # tránh lặp điểm đầu

                path_list_dense.extend(segment_pts)

        return path_list_dense

    def create_arc(self, p_start, p_mid, p_end, step_size=0.02):
    # Tính tâm và bán kính đường tròn
        def circle_center(a, b, c):
            A = b - a
            B = c - a
            A_perp = np.array([-A[1], A[0]])
            B_perp = np.array([-B[1], B[0]])
            midA = (a + b) / 2
            midB = (a + c) / 2
            M = np.array([A_perp, -B_perp]).T
            if np.linalg.matrix_rank(M) < 2:
                return None, None
            rhs = midB - midA
            t = np.linalg.solve(M, rhs)
            O = midA + A_perp * t[0]
            return O, np.linalg.norm(O - a)

        O, R = circle_center(p_start, p_mid, p_end)
        if O is None:
            return [p_start.tolist(), p_end.tolist()]

        v1 = p_start - O
        v2 = p_end - O
        angle1 = np.arctan2(v1[1], v1[0])
        angle2 = np.arctan2(v2[1], v2[0])

        angle_diff = angle2 - angle1
        if angle_diff <= -np.pi:
            angle_diff += 2 * np.pi
        elif angle_diff > np.pi:
            angle_diff -= 2 * np.pi

        arc_len = abs(R * angle_diff)
        n_steps = max(2, int(np.ceil(arc_len / step_size)))
        angles = np.linspace(angle1, angle2, n_steps)

        arc_pts = [(O + R * np.array([np.cos(a), np.sin(a)])).tolist() for a in angles]
        arc_pts.append(p_end.tolist())

        return arc_pts

    def handle_provide_path_request(self, req):
        """Hàm xử lý chính khi nhận được yêu cầu từ client."""
        robot_name = req.robot_name
        #rospy.loginfo(f"Nhận được yêu cầu tạo path cho '{robot_name}'.")

        if robot_name not in self.robot_poses:
            error_msg = f"Không có thông tin vị trí cho '{robot_name}'."
            rospy.logerr(error_msg)
            return ProvidePathResponse(path=Path(), success=False, message=error_msg)

        start_pose = self.robot_poses[robot_name]
        start_pos = (start_pose[0], start_pose[1])
        path_points = self.generate_final_path(start_pos)
        
        if not path_points:
             rospy.logwarn(f"Không thể tạo path hợp lệ cho {robot_name}.")
             return ProvidePathResponse(path=Path(), success=False, message="Tạo path không thành công.")
        
        # Chuyển đổi dữ liệu điểm thành message nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = rospy.Time.now()
        for p in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        pub = self.get_publisher_for_robot(req.robot_name)
        pub.publish(path_msg)
        #rospy.loginfo(f"Đã tạo path mới cho '{robot_name}'. Đang gửi trả lại cho client.")
        # Trả về quỹ đạo trong response
        return ProvidePathResponse(path=path_msg, success=True, message="Cung cấp path thành công.")

if __name__ == '__main__':
    rospy.init_node('path_provider_server_node')
    try:
        server = PathProviderServer()
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(f"Lỗi khởi tạo node Path Provider: {e}")