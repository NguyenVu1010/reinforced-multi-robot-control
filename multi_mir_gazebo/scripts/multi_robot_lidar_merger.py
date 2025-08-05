#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import threading
import math
import copy

class MultiRobotLidarMerger:
    def __init__(self):
        rospy.init_node("multi_robot_lidar_merger_node")

        # Danh sách các robot cần xử lý
        self.robot_list = rospy.get_param("~robot_list", ["robot1", "robot2", "robot3"])

        self.lock = threading.Lock()
        self.f_scan_data = {}  # {robot_name: LaserScan}
        self.b_scan_data = {}

        self.publishers = {}

        for robot in self.robot_list:
            rospy.Subscriber(f"/{robot}/f_scan", LaserScan, self.make_cb(robot, "front"))
            rospy.Subscriber(f"/{robot}/b_scan", LaserScan, self.make_cb(robot, "back"))
            self.publishers[robot] = rospy.Publisher(f"/{robot}/scan", LaserScan, queue_size=1)

        rospy.loginfo(f"[LidarMerger] Đã khởi tạo cho robots: {self.robot_list}")

    def make_cb(self, robot_name, direction):
        def cb(msg):
            with self.lock:
                if direction == "front":
                    self.f_scan_data[robot_name] = msg
                else:
                    self.b_scan_data[robot_name] = msg
                self.try_merge(robot_name)
        return cb

    def try_merge(self, robot):
        if robot not in self.f_scan_data or robot not in self.b_scan_data:
            return

        f_msg = self.f_scan_data[robot]
        b_msg = self.b_scan_data[robot]

        # Đảo ngược b_scan (quay 180 độ)
        b_ranges_rotated = list(reversed(b_msg.ranges))

        # Ghép ranges
        merged_ranges = list(f_msg.ranges) + b_ranges_rotated

        # Tạo message mới
        merged = LaserScan()
        merged.header.stamp = rospy.Time.now()
        merged.header.frame_id = f_msg.header.frame_id  # Giả định cùng frame

        merged.angle_min = -math.pi
        merged.angle_max = math.pi
        merged.angle_increment = f_msg.angle_increment  # Giả định giống nhau
        merged.range_min = min(f_msg.range_min, b_msg.range_min)
        merged.range_max = max(f_msg.range_max, b_msg.range_max)
        merged.scan_time = f_msg.scan_time
        merged.time_increment = f_msg.time_increment
        merged.ranges = merged_ranges
        merged.intensities = []  # hoặc xử lý nếu có

        self.publishers[robot].publish(merged)

if __name__ == "__main__":
    try:
        MultiRobotLidarMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
