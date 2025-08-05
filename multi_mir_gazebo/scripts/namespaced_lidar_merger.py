#!/usr/bin/env python3

"""
A ROS node to merge two LaserScan topics by performing manual, point-by-point
transformations, avoiding the creation of an intermediate PointCloud2 message.
"""

import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs  # Crucial for transforming PointStamped
from sensor_msgs.msg import LaserScan
import threading
import math
import numpy as np
import copy

class ManualTransformLidarMerger:
    def __init__(self):
        rospy.init_node("lidar_merger_manual_transform")

        namespace = rospy.get_namespace()
        self.robot_name = namespace.strip('/')
        if not self.robot_name:
            rospy.logfatal("LidarMerger must be run inside a robot's namespace. Shutting down.")
            rospy.signal_shutdown("No namespace specified.")
            return

        rospy.loginfo(f"--- Manual Transform Lidar Merger initializing for robot: '{self.robot_name}' ---")
        
        self.base_frame = f"{self.robot_name}_base_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.f_scan_data = None
        self.b_scan_data = None
        self.data_lock = threading.Lock()

        rospy.Subscriber("f_scan", LaserScan, self.f_scan_callback)
        rospy.Subscriber("b_scan", LaserScan, self.b_scan_callback)
        self.publisher = rospy.Publisher("scan", LaserScan, queue_size=2)

        self.processing_timer = rospy.Timer(rospy.Duration(0.05), self.merge_loop)

        rospy.loginfo(f"[{self.robot_name}] Setup complete. Waiting for data...")

    def f_scan_callback(self, msg):
        with self.data_lock:
            self.f_scan_data = msg

    def b_scan_callback(self, msg):
        with self.data_lock:
            self.b_scan_data = msg

    def merge_loop(self, event=None):
        f_msg, b_msg = None, None
        
        with self.data_lock:
            if self.f_scan_data is not None and self.b_scan_data is not None:
                f_msg = copy.deepcopy(self.f_scan_data)
                b_msg = copy.deepcopy(self.b_scan_data)
                self.f_scan_data = None
                self.b_scan_data = None
            else:
                return

        try:
            # 1. Chuẩn bị một message LaserScan 360 độ rỗng
            latest_stamp = max(f_msg.header.stamp, b_msg.header.stamp)
            merged_scan = self.create_empty_360_scan(latest_stamp, f_msg, b_msg)
            if merged_scan is None: return

            # 2. Điền dữ liệu từ Lidar trước vào scan rỗng
            self.fill_scan_from_source(f_msg, merged_scan)
            
            # 3. Điền dữ liệu từ Lidar sau vào scan rỗng
            self.fill_scan_from_source(b_msg, merged_scan)

            # 4. Xuất bản kết quả
            self.publisher.publish(merged_scan)

        except (tf2_ros.ConnectivityException, tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[{self.robot_name}] TF Error in merge loop (will retry): {e}")
            return
        except Exception as e:
            rospy.logerr(f"[{self.robot_name}] An unexpected error occurred in merge loop: {e}")
            
    def create_empty_360_scan(self, stamp, f_msg, b_msg):
        """Tạo một message LaserScan rỗng với các thông số hợp nhất."""
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = self.base_frame
        
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        
        # Sử dụng độ phân giải góc tốt hơn từ hai Lidar (nhỏ hơn)
        scan.angle_increment = min(f_msg.angle_increment, b_msg.angle_increment)
        if scan.angle_increment == 0:
            rospy.logerr_once(f"[{self.robot_name}] Angle increment is zero, cannot create scan.")
            return None

        scan.range_min = min(f_msg.range_min, b_msg.range_min)
        scan.range_max = max(f_msg.range_max, b_msg.range_max)
        
        num_scan_points = int(math.ceil((scan.angle_max - scan.angle_min) / scan.angle_increment))
        scan.ranges = [float('inf')] * num_scan_points
        return scan

    def fill_scan_from_source(self, source_scan, target_scan):
        """
        Lặp qua từng điểm của source_scan, biến đổi nó, và điền vào target_scan.
        """
        source_frame = source_scan.header.frame_id
        
        # Lấy phép biến đổi từ frame của Lidar nguồn sang frame base_link
        # Chỉ cần làm một lần cho cả scan để tiết kiệm tài nguyên
        transform = self.tf_buffer.lookup_transform(
            target_scan.header.frame_id, # đích, ví dụ: "robot1/base_link"
            source_frame,               # nguồn, ví dụ: "robot1/f_scan_link"
            source_scan.header.stamp,   # tại thời điểm của scan
            rospy.Duration(0.1)
        )

        # Lặp qua từng điểm đo trong Lidar nguồn
        for i, r in enumerate(source_scan.ranges):
            # Bỏ qua các giá trị không hợp lệ
            if not (source_scan.range_min <= r <= source_scan.range_max):
                continue

            # Bước 1: Chuyển (range, angle) thành (x, y) trong frame của Lidar nguồn
            angle_in_source = source_scan.angle_min + i * source_scan.angle_increment
            x_source = r * math.cos(angle_in_source)
            y_source = r * math.sin(angle_in_source)

            # Bước 2: Tạo một đối tượng PointStamped để tf2 có thể biến đổi
            point_in_source_frame = PointStamped()
            point_in_source_frame.header = source_scan.header
            point_in_source_frame.point.x = x_source
            point_in_source_frame.point.y = y_source
            point_in_source_frame.point.z = 0.0 # Giả định Lidar 2D

            # Bước 3: Biến đổi điểm đó sang frame đích (base_link)
            point_in_target_frame = tf2_geometry_msgs.do_transform_point(point_in_source_frame, transform)

            # Bước 4: Chuyển điểm đã biến đổi ngược lại thành (range', angle')
            x_target = point_in_target_frame.point.x
            y_target = point_in_target_frame.point.y
            
            range_in_target = math.sqrt(x_target**2 + y_target**2)
            angle_in_target = math.atan2(y_target, x_target)

            # Bước 5: Tìm index và điền vào mảng ranges của scan đích
            if target_scan.range_min <= range_in_target <= target_scan.range_max:
                index = int((angle_in_target - target_scan.angle_min) / target_scan.angle_increment)
                num_points = len(target_scan.ranges)

                if 0 <= index < num_points:
                    # Xử lý vùng trùng lặp: luôn lấy khoảng cách nhỏ nhất
                    if target_scan.ranges[index] > range_in_target:
                        target_scan.ranges[index] = range_in_target

if __name__ == "__main__":
    try:
        ManualTransformLidarMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass