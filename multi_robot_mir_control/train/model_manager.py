# trainer_logic/model_manager.py

import rospy
import torch
import os
import pickle

# Giả định các lớp này nằm trong thư mục `models` và `utils` của bạn
from models.policy import PolicyNetwork
from models.value import ValueNetwork
from utils.logger import setup_save_dir, save_model
from utils.normalizer import ObservationNormalizer

class ModelManager:
    """
    Quản lý việc khởi tạo, tải, lưu model và normalizer.
    """
    def __init__(self, model_cfg, env_cfg, checkpoint_cfg, save_dir):
        self.model_cfg = model_cfg
        self.env_cfg = env_cfg
        self.checkpoint_cfg = checkpoint_cfg
        self.save_dir = setup_save_dir(save_dir) # Hàm này sẽ tạo thư mục nếu chưa có
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        rospy.loginfo(f"ModelManager sử dụng thiết bị: {self.device}")

        # Khởi tạo các thành phần
        self.policy_net = PolicyNetwork(env_cfg['state_dim'], env_cfg['action_dim'], model_cfg['hidden_dim']).to(self.device)
        self.value_net = ValueNetwork(env_cfg['state_dim'], model_cfg['hidden_dim']).to(self.device)
        self.obs_normalizer = ObservationNormalizer(num_inputs=env_cfg['state_dim'])
        
    def load_checkpoints(self):
        """Tải các model và normalizer từ checkpoint nếu được cấu hình."""
        start_cycle = 0
        if not self.checkpoint_cfg.get('load_from_checkpoint', False):
            rospy.loginfo("Bắt đầu training từ các trọng số ngẫu nhiên.")
            return start_cycle

        rospy.loginfo("Đang tải model từ checkpoint...")
        # Tải Policy
        policy_path = self.checkpoint_cfg.get('policy_checkpoint_path')
        if policy_path and os.path.exists(policy_path):
            try:
                self.policy_net.load_state_dict(torch.load(policy_path, map_location=self.device))
                rospy.loginfo(f"Đã tải Policy từ: {policy_path}")
                # Trích xuất số chu kỳ từ tên file để tiếp tục đếm
                start_cycle = int(policy_path.split('_')[-1].split('.')[0])
            except (ValueError, IndexError, Exception) as e:
                rospy.logerr(f"Lỗi khi tải checkpoint Policy: {e}. Bắt đầu từ đầu.")
                start_cycle = 0
        else:
            rospy.logwarn(f"Không tìm thấy file Policy checkpoint tại: {policy_path}")

        # Tải Value
        value_path = self.checkpoint_cfg.get('value_checkpoint_path')
        if value_path and os.path.exists(value_path):
            try:
                self.value_net.load_state_dict(torch.load(value_path, map_location=self.device))
                rospy.loginfo(f"Đã tải Value từ: {value_path}")
            except Exception as e:
                rospy.logerr(f"Lỗi khi tải checkpoint Value: {e}")

        # Tải Normalizer
        normalizer_path = self.checkpoint_cfg.get('normalizer_checkpoint_path')
        if normalizer_path and os.path.exists(normalizer_path):
            try:
                with open(normalizer_path, 'rb') as f:
                    self.obs_normalizer = pickle.load(f)
                rospy.loginfo(f"Đã tải Normalizer từ: {normalizer_path}")
            except Exception as e:
                rospy.logerr(f"Lỗi khi tải checkpoint Normalizer: {e}")
        
        if start_cycle > 0:
            rospy.loginfo(f"Sẽ tiếp tục huấn luyện từ chu kỳ {start_cycle + 1}")
        return start_cycle
        
    def save_checkpoint(self, cycle):
        """Lưu checkpoint hiện tại."""
        rospy.loginfo(f"Lưu checkpoint tại chu kỳ {cycle}...")
        save_model(self.policy_net, self.save_dir, f'policy_net_cycle_{cycle}.pth')
        save_model(self.value_net, self.save_dir, f'value_net_cycle_{cycle}.pth')
        with open(os.path.join(self.save_dir, f'obs_normalizer_cycle_{cycle}.pkl'), 'wb') as f:
            pickle.dump(self.obs_normalizer, f)
        rospy.loginfo("Lưu checkpoint hoàn tất.")

    def save_final_models(self, final_cycle_count):
        """Lưu model cuối cùng khi kết thúc huấn luyện."""
        rospy.loginfo("Lưu model cuối cùng...")
        save_model(self.policy_net, self.save_dir, f'policy_net_final_{final_cycle_count}.pth')
        save_model(self.value_net, self.save_dir, f'value_net_final_{final_cycle_count}.pth')
        with open(os.path.join(self.save_dir, 'obs_normalizer_final.pkl'), 'wb') as f:
            pickle.dump(self.obs_normalizer, f)
        rospy.loginfo("Lưu model cuối cùng hoàn tất.")