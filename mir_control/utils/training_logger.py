# utils/training_logger.py

import csv
import os
import rospy
from datetime import datetime

class TrainingLogger:
    """
    Một lớp chuyên dụng để ghi lại dữ liệu huấn luyện ra file CSV.
    """
    def __init__(self, save_dir, robot_names,filename='training_log.csv'):
        """
        Khởi tạo logger.
        
        Args:
            save_dir (str): Thư mục để lưu file log.
            robot_names (list of str): Danh sách tên của các robot.
        """
        self.robot_names = robot_names
        
        # Tạo một tên file duy nhất dựa trên thời gian bắt đầu
        self.filepath = os.path.join(save_dir,filename)
        
        # Mở file và giữ lại file handle
        # `newline=''` là rất quan trọng để tránh các dòng trống trong file CSV
        self.file_handle = open(self.filepath, 'a', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        # Ghi dòng header
        self._write_header()
        
        rospy.loginfo(f"[TrainingLogger] Đã tạo file log tại: {self.filepath}")

    def _write_header(self):
        """Ghi dòng tiêu đề cho file CSV."""
        header = ['Cycle']
        # Thêm các cột cho từng robot
        for name in self.robot_names:
            header.append(f'{name}_Total_Reward')
            header.append(f'{name}_Avg_Reward_per_Step')
        
        # Thêm các cột tổng hợp
        header.append('System_Total_Reward')
        header.append('System_Avg_Reward_per_Step')
        
        self.csv_writer.writerow(header)
        self.file_handle.flush() # Đảm bảo header được ghi ngay lập tức

    def log_cycle_data(self, cycle_num, total_rewards_per_robot, avg_rewards_per_robot,
                       system_total_reward, system_avg_reward):
        """
        Ghi dữ liệu của một chu kỳ huấn luyện vào file.
        
        Args:
            cycle_num (int): Số thứ tự của chu kỳ hiện tại.
            total_rewards_per_robot (dict): {'robot1': 100, ...}
            avg_rewards_per_robot (dict): {'robot1': 0.1, ...}
            system_total_reward (float): Tổng reward của cả hệ thống.
            system_avg_reward (float): Reward trung bình của cả hệ thống.
        """
        row = [cycle_num]
        
        # Thêm dữ liệu của từng robot vào hàng
        for name in self.robot_names:
            row.append(f"{total_rewards_per_robot.get(name, 0):.2f}")
            row.append(f"{avg_rewards_per_robot.get(name, 0):.4f}")
            
        # Thêm dữ liệu tổng hợp
        row.append(f"{system_total_reward:.2f}")
        row.append(f"{system_avg_reward:.4f}")
        
        # Ghi hàng vào file
        self.csv_writer.writerow(row)
        self.file_handle.flush() # Ghi dữ liệu ra đĩa ngay để tránh mất mát nếu crash

    def close(self):
        """Đóng file một cách an toàn."""
        if self.file_handle:
            rospy.loginfo(f"[TrainingLogger] Đang đóng file log: {self.filepath}")
            self.file_handle.close()
            self.file_handle = None