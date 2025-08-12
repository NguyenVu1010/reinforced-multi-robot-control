# utils/normalizer.py

import numpy as np
import pickle

class ObservationNormalizer:
    # --- THAY ĐỔI Ở ĐÂY: Chỉ nhận một tham số `num_inputs` ---
    def __init__(self, num_inputs, clip_obs=10.0):
        self.num_inputs = num_inputs
        
        # --- Gán giá trị cố định cho clip_obs ---
        self.clip_obs = clip_obs
        
        # Các khởi tạo khác giữ nguyên
        self.mean = np.zeros(num_inputs, dtype=np.float64)
        self.std = np.ones(num_inputs, dtype=np.float64)
        self.variance = np.ones(num_inputs, dtype=np.float64)
        self.count = 1e-4

    def observe(self, x):
        """Cập nhật running mean và std."""
        x = np.atleast_2d(x)
        batch_count = x.shape[0]
        batch_mean = np.mean(x, axis=0)
        batch_variance = np.var(x, axis=0)
        
        delta = batch_mean - self.mean
        new_count = self.count + batch_count
        
        self.mean += delta * batch_count / new_count
        
        m_a = self.variance * self.count
        m_b = batch_variance * batch_count
        m_2 = m_a + m_b + np.square(delta) * self.count * batch_count / new_count
        self.variance = m_2 / new_count
        
        self.count = new_count
        self.std = np.sqrt(self.variance + 1e-8)

    def normalize(self, inputs):
        """
        Chuẩn hóa inputs. Inputs có thể là một vector đơn (1D) 
        hoặc một batch các vector (2D).
        """
        mean_f32 = self.mean.astype(np.float32)
        std_f32 = self.std.astype(np.float32)
        
        normalized_inputs = (inputs - mean_f32) / std_f32
        
        # Dòng này bây giờ sẽ hoạt động vì self.clip_obs đã được định nghĩa trong __init__
        clipped_inputs = np.clip(normalized_inputs, -self.clip_obs, self.clip_obs)
        
        return clipped_inputs

    def save_state(self, filepath):
        """Lưu trạng thái của normalizer."""
        # Lưu cả clip_obs để đảm bảo tính nhất quán khi tải lại
        state = {
            'mean': self.mean, 
            'std': self.std, 
            'count': self.count, 
            'variance': self.variance,
            'clip_obs': self.clip_obs 
        }
        with open(filepath, 'wb') as f:
            pickle.dump(state, f)

    def load_state(self, filepath):
        """Tải trạng thái của normalizer."""
        with open(filepath, 'rb') as f:
            state = pickle.load(f)
        self.mean = state['mean']
        self.std = state['std']
        self.count = state.get('count', 1e-4)
        self.variance = state.get('variance', np.square(self.std))
        # Tải clip_obs nếu có, nếu không thì dùng giá trị mặc định
        self.clip_obs = state.get('clip_obs', 10.0)