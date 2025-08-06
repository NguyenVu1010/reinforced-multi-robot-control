#!/usr/bin/env python3

"""
path_provider_service_node.py

ROS Node cung cấp một service để tìm đường đi cho robot trong môi trường Gazebo.
Node này sử dụng thuật toán A* trên một đồ thị định trước (pre-defined graph).

Chức năng chính:
1.  Tải một đồ thị các điểm (nodes) và các cạnh (edges) từ file JSON.
2.  Đọc cấu hình bản đồ (kích thước ảnh, độ phân giải) từ file YAML.
3.  **Quan trọng**: Chuyển đổi tất cả tọa độ trong đồ thị từ hệ pixel của ảnh bản đồ
    sang hệ tọa độ thực (map) của ROS/Gazebo ngay khi khởi tạo.
4.  Cung cấp một ROS Service 'request_path_service' nhận tên robot và điểm đến.
5.  Sử dụng vị trí hiện tại của robot và điểm đến để tìm đường đi tối ưu trên đồ thị.
6.  Ghép nối các đoạn đường đi chi tiết (pre-defined paths) giữa các node để tạo
    thành một đường đi hoàn chỉnh và mượt mà.
7.  Xuất bản (publish) đường đi cuối cùng dưới dạng nav_msgs/Path.

Sửa lỗi so với phiên bản trước:
-   Đã khắc phục lỗi nghiêm trọng về việc trộn lẫn hệ tọa độ pixel và hệ tọa độ map.
    Tất cả dữ liệu giờ được chuẩn hóa về hệ tọa độ map.
-   Đọc độ phân giải 'resolution' từ file config thay vì hardcode.
"""

import rospy
import numpy as np
import os
import heapq
import json
import math
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from mir_control.srv import RequestPath, RequestPathResponse, RequestPathRequest

class AStarPathFinder:
    def __init__(self, nodes_data, edges_data):
        self.nodes = nodes_data
        self.graph = {}
        for edge in edges_data:
            if edge['from'] in self.nodes and edge['to'] in self.nodes:
                # Tạo đồ thị có hướng
                self.graph.setdefault(edge['from'], []).append((edge['to'], edge['length']))
        rospy.loginfo(f"A* PathFinder được khởi tạo với {len(self.nodes)} nodes và {len(edges_data)} cạnh có hướng.")
        rospy.loginfo("Tất cả tọa độ trong A* đều ở hệ tọa độ MAP.")

    def find_nearest_node(self, position):
        if not self.nodes:
            return None
        # Tính khoảng cách Euclidean trong không gian 2D
        distances = {name: math.dist(position, coords) for name, coords in self.nodes.items()}
        return min(distances, key=distances.get)

    def heuristic(self, node_a, node_b):
        pos_a = self.nodes[node_a]
        pos_b = self.nodes[node_b]
        return math.dist(pos_a, pos_b)

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.insert(0, current)
        return total_path

    def find_path(self, start_node, end_node):
        if start_node not in self.nodes or end_node not in self.nodes:
            rospy.logerr(f"Node '{start_node}' hoặc '{end_node}' không tồn tại trong đồ thị.")
            return None, float('inf')
            
        open_set = [(0, start_node)]
        came_from = {}
        g_score = {node: float('inf') for node in self.nodes}
        g_score[start_node] = 0
        f_score = {node: float('inf') for node in self.nodes}
        f_score[start_node] = self.heuristic(start_node, end_node)

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == end_node:
                path = self.reconstruct_path(came_from, current)
                return path, g_score[end_node]
                
            for neighbor, cost in self.graph.get(current, []):
                tentative_g_score = g_score[current] + cost
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, end_node)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
                    
        rospy.logwarn(f"A* không tìm thấy đường đi từ '{start_node}' đến '{end_node}'.")
        return None, float('inf')


class UnifiedPathProvider:
    def __init__(self, config_file_path='src/mir_robot/mir_control/config/config.yaml'):
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.robot_names = ['robot1', 'robot2', 'robot3']
        self.map_point_density = rospy.get_param('~map_point_density', 50.0) # điểm / mét
        self.path_publishers = {name: rospy.Publisher(f'/paths/{name}/planned_path', Path, queue_size=1, latch=True) for name in self.robot_names}
        self.path_library_path = rospy.get_param('~path_library_path')
        if not self.path_library_path or not os.path.exists(self.path_library_path):
            rospy.logfatal(f"Tham số 'path_library_path' không được set hoặc file không tồn tại: {self.path_library_path}")
            rospy.signal_shutdown("Lỗi file path library JSON.")
            return

        self.robot_poses = {}
        self.path_finder = None
        self.edges_dict = {}

        try:
            # BƯỚC 1: Tải cấu hình bản đồ (kích thước ảnh, độ phân giải)
            with open(config_file_path, 'r') as f:
                config = yaml.safe_load(f)
            self.image_width = config['map_config']['map_width']
            self.image_height = config['map_config']['map_height']
            self.resolution = config['map_config']['resolution'] # Vd: 0.05 mét/pixel
            rospy.loginfo(f"Tải cấu hình map: W={self.image_width}, H={self.image_height}, Res={self.resolution} m/pixel")

            # BƯỚC 2: Tải dữ liệu đồ thị từ file JSON
            with open(self.path_library_path, 'r', encoding='utf-8') as f:
                graph_data = json.load(f)

            # BƯỚC 3: CHUYỂN ĐỔI TỌA ĐỘ TỪ PIXEL SANG MAP (MÉT)
            # Chuyển đổi tọa độ của NODES
            nodes_data_pixel = graph_data.get('nodes', {})
            nodes_data_map = {}
            for name, coords_pixel in nodes_data_pixel.items():
                x_map, y_map = self.convert_pixel_to_cartesian(
                    coords_pixel[0], coords_pixel[1], 
                    self.image_width, self.image_height, self.resolution
                )
                nodes_data_map[name] = (x_map, y_map)
            
            edges_data = graph_data.get('edges', [])
            
            # Khởi tạo A* PathFinder với dữ liệu đã được chuyển đổi sang hệ tọa độ map
            self.path_finder = AStarPathFinder(nodes_data_map, edges_data)

            # Chuyển đổi tọa độ của các ĐIỂM TRÊN CẠNH (path_points)
            for edge in edges_data:
                key = (edge['from'], edge['to'])
                path_points_pixel = edge.get('path_points', [])
                path_points_map = []
                for point_pixel in path_points_pixel:
                    x_map, y_map = self.convert_pixel_to_cartesian(
                        point_pixel[0], point_pixel[1], 
                        self.image_width, self.image_height, self.resolution
                    )
                    path_points_map.append([x_map, y_map])
                self.edges_dict[key] = path_points_map

        except Exception as e:
            rospy.logfatal(f"Không thể đọc hoặc xử lý file path_library.json hoặc config.yaml: {e}")
            rospy.signal_shutdown("Lỗi file JSON hoặc YAML.")
            return

        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.service = rospy.Service('request_path_service', RequestPath, self.handle_path_request)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        rospy.loginfo("Path Provider Service (Đồ thị có hướng đơn) đã sẵn sàng.")

    @staticmethod
    def convert_pixel_to_cartesian(x_pixel, y_pixel, image_width, image_height, resolution):
        # Tâm của ảnh (hệ tọa độ pixel)
        x_center_pixel = image_width / 2.0
        y_center_pixel = image_height / 2.0
        
        # Chuyển gốc tọa độ về tâm ảnh
        # Trục y của ảnh ngược với trục y của hệ Descartes
        x_from_center = x_pixel - x_center_pixel
        y_from_center = y_center_pixel - y_pixel 
        
        # Chuyển đổi từ pixel sang mét bằng độ phân giải
        x_real = x_from_center * resolution
        y_real = y_from_center * resolution
        
        return x_real, y_real

    def model_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            if 'robot' in name:
                pose_msg = msg.pose[i]
                ori = pose_msg.orientation
                _, _, yaw = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
                self.robot_poses[name] = (pose_msg.position.x, pose_msg.position.y, yaw)

    def handle_path_request(self, req: RequestPathRequest):
        rospy.loginfo(f"Nhận yêu cầu tìm đường cho robot '{req.robot_name}'.")

        if req.robot_name not in self.robot_poses:
            msg = f"Không tìm thấy vị trí của robot '{req.robot_name}'."
            rospy.logerr(msg)
            return RequestPathResponse(success=False, message=msg)
            
        # Tất cả tọa độ giờ đây đều là tọa độ map
        start_pos = self.robot_poses[req.robot_name][:2]
        end_pos = (req.end_point.x, req.end_point.y)

        start_node = self.path_finder.find_nearest_node(start_pos)
        end_node = self.path_finder.find_nearest_node(end_pos)

        if not start_node or not end_node:
            msg = "Không thể tìm thấy node gần nhất cho vị trí bắt đầu hoặc kết thúc."
            rospy.logerr(msg)
            return RequestPathResponse(success=False, message=msg)

        # rospy.loginfo(f"Vị trí bắt đầu (map): {start_pos} -> Node vào: '{start_node}'")
        # rospy.loginfo(f"Vị trí kết thúc (map): {end_pos} -> Node ra: '{end_node}'")

        main_path_keys, _ = self.path_finder.find_path(start_node, end_node)

        if not main_path_keys:
            msg = f"A* không tìm thấy đường đi từ '{start_node}' đến '{end_node}'."
            rospy.logwarn(msg)
            return RequestPathResponse(success=False, message=msg)

        # Ghép nối các đoạn đường đi. Tất cả các điểm đều ở tọa độ map.
        start_node_pos = self.path_finder.nodes[start_node]
        full_path_points = self._interpolate_line(start_pos, start_node_pos, self.map_point_density)
        for i in range(len(main_path_keys) - 1):
            from_n, to_n = main_path_keys[i], main_path_keys[i+1]
            segment_points = self.edges_dict.get((from_n, to_n))
            
            if segment_points:
                # Nếu đường đi đã có điểm, bỏ qua điểm đầu tiên của đoạn mới
                # để tránh trùng lặp với điểm cuối của đoạn trước.
                if len(full_path_points) > 0 and len(segment_points) > 0:
                    full_path_points.extend(segment_points[1:])
                else:
                    full_path_points.extend(segment_points)
            else:
                # Fallback: nếu không có đường đi chi tiết, chỉ cần đi đến node tiếp theo
                rospy.logwarn(f"Không tìm thấy đoạn đường chi tiết {from_n}->{to_n} trong thư viện. Sử dụng vị trí node.")
                full_path_points.append(list(self.path_finder.nodes[to_n]))

        end_node_pos = self.path_finder.nodes[end_node]
        exit_segment = self._interpolate_line(end_node_pos, end_pos, self.map_point_density)
        if len(exit_segment) > 1:
            # Dùng extend và bỏ đi điểm đầu tiên để tránh trùng lặp
            full_path_points.extend(exit_segment[1:])

        path_msg = self.points_to_path_msg(full_path_points)
        self.path_publishers[req.robot_name].publish(path_msg)
        self.path_pub.publish(path_msg)

        success_msg = f"[{req.robot_name}]Đã tạo đường đi hoàn chỉnh cho với {len(full_path_points)} điểm."
        rospy.loginfo(success_msg)
        return RequestPathResponse(path=path_msg, success=True, message=success_msg)
    def _interpolate_line(self, p1, p2, density):
        p1_np, p2_np = np.array(p1), np.array(p2); length = np.linalg.norm(p2_np - p1_np)
        num_points = max(2, int(length * density)); points = np.linspace(p1_np, p2_np, num_points)
        return [p.tolist() for p in points]
    def points_to_path_msg(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()
        for p in path_points:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            pose.pose.orientation.w = 1.0  # Hướng mặc định
            path_msg.poses.append(pose)
        return path_msg

if __name__ == '__main__':
    rospy.init_node('path_provider_service_node')
    try:
        # Nhớ cập nhật đường dẫn tới file config của bạn
        server = UnifiedPathProvider(config_file_path='/home/nguyen1/catkin_ws/src/mir_robot/mir_control/config/config.yaml')
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logerr(f"Lỗi khởi tạo node Path Provider: {e}")
    except Exception as e:
        rospy.logfatal(f"Lỗi không mong muốn trong quá trình khởi tạo: {e}")