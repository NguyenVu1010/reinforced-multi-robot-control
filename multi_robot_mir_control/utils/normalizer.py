# utils/normalizer.py

import numpy as np

class ObservationNormalizer:
    """
    Theo dõi và chuẩn hóa các quan sát bằng cách sử dụng
    giá trị trung bình và độ lệch chuẩn động (running mean/std).
    Sử dụng thuật toán Welford để cập nhật một cách ổn định.
    """
    def __init__(self, num_inputs):
        self.n = 0
        self.mean = np.zeros(num_inputs, dtype=np.float64)
        self.M2 = np.zeros(num_inputs, dtype=np.float64)
        self.std = np.ones(num_inputs, dtype=np.float64) # Bắt đầu với std=1 để tránh chia cho 0

    def observe(self, x):
        """Cập nhật mean và std với một quan sát mới x."""
        x = np.asarray(x)
        self.n += 1
        delta = x - self.mean
        self.mean += delta / self.n
        delta2 = x - self.mean
        self.M2 += delta * delta2
        
        if self.n > 1:
            self.std = np.sqrt(self.M2 / (self.n - 1))

    def normalize(self, inputs):
        """Chuẩn hóa một hoặc nhiều quan sát."""
        # Thêm hằng số nhỏ để tránh chia cho 0
        epsilon = 1e-8
        
        # Xử lý cả trường hợp đầu vào là một vector hoặc một batch các vector
        obs_mean = self.mean
        obs_std = self.std
        if inputs.ndim > 1:
            # Nếu là batch, cần reshape mean và std để khớp
            obs_mean = np.expand_dims(self.mean, axis=0)
            obs_std = np.expand_dims(self.std, axis=0)

        normalized = (inputs - obs_mean) / (obs_std + epsilon)
        
        # Cắt giá trị để tránh các giá trị ngoại lai quá lớn sau khi chuẩn hóa
        return np.clip(normalized, -5.0, 5.0)