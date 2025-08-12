# trainer_logic/config_manager.py

import yaml
import os
import rospkg
import rospy

class ConfigManager:
    """
    Quản lý việc tải và truy cập các thông số cấu hình từ file YAML.
    """
    def __init__(self, config_path):
        if not os.path.exists(config_path):
            rospy.logfatal(f"File cấu hình không tồn tại tại: {config_path}")
            raise FileNotFoundError(f"File cấu hình không tồn tại tại: {config_path}")

        rospy.loginfo(f"Đang tải file cấu hình từ: {config_path}")
        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f)

        # Phân tách các mục cấu hình để dễ truy cập
        self.env_cfg = self.config['env']
        self.train_cfg = self.config['train']
        self.model_cfg = self.config['model']
        self.checkpoint_cfg = self.config.get('checkpoint', {})

        # Xác định các đường dẫn quan trọng một cách an toàn
        try:
            rospack = rospkg.RosPack()
            # Lấy tên package từ chính file config để linh hoạt
            pkg_name = self.config.get('package_name', 'mir_control')
            self.pkg_path = rospack.get_path(pkg_name)
        except rospkg.ResourceNotFound:
            rospy.logfatal(f"Không thể tìm thấy ROS package '{pkg_name}'. Hãy kiểm tra tên trong config.yaml và đảm bảo workspace đã được source.")
            raise

        self.save_dir = os.path.join(self.pkg_path, "saved_models")
        self.pause_file = os.path.join(self.pkg_path, "PAUSE")
        self.stop_file = os.path.join(self.pkg_path, "STOP")