# trainer_logic/system_controller.py

import rospy
import os
import time

class SystemController:
    """
    Xử lý các tương tác với hệ thống như file điều khiển PAUSE/STOP.
    """
    def __init__(self, pause_file_path, stop_file_path):
        self.pause_file = pause_file_path
        self.stop_file = stop_file_path
        rospy.loginfo(f"File PAUSE được theo dõi tại: {self.pause_file}")
        rospy.loginfo(f"File STOP được theo dõi tại: {self.stop_file}")

    def cleanup_control_files(self):
        """Xóa các file điều khiển cũ khi khởi động."""
        try:
            if os.path.exists(self.pause_file): os.remove(self.pause_file)
            if os.path.exists(self.stop_file): os.remove(self.stop_file)
        except OSError as e:
            rospy.logwarn(f"Không thể xóa file điều khiển cũ: {e}")

    def check_for_signals(self):
        """
        Kiểm tra các file điều khiển và hành động tương ứng.
        Returns:
            bool: True nếu nhận được tín hiệu dừng, False nếu tiếp tục.
        """
        if os.path.exists(self.pause_file):
            rospy.loginfo("Phát hiện file PAUSE. Tạm dừng huấn luyện...")
            rospy.loginfo(f"Để tiếp tục, hãy xóa file: '{self.pause_file}'")
            while os.path.exists(self.pause_file) and not rospy.is_shutdown():
                time.sleep(2)
            if not rospy.is_shutdown():
                rospy.loginfo("File PAUSE đã được xóa. Tiếp tục huấn luyện.")

        if os.path.exists(self.stop_file):
            rospy.logwarn("Phát hiện file STOP. Kết thúc huấn luyện sớm...")
            try:
                os.remove(self.stop_file)
            except OSError as e:
                rospy.logerr(f"Không thể xóa file STOP: {e}")
            return True  # Tín hiệu dừng

        return False  # Tín hiệu tiếp tục