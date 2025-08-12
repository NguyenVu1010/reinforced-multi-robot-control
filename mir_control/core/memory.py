# core/memory.py

import numpy as np

class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
        self.dones = []
        self.log_probs = []

    def clear(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]
        del self.dones[:]
        del self.log_probs[:]

    def add(self, state, action, reward, done, log_prob):
        self.states.append(state)
        self.actions.append(action)
        self.rewards.append(reward)
        self.is_terminals.append(done)
        self.logprobs.append(log_prob)
        self.dones.append(done)
        self.log_probs.append(log_prob)
    def __len__(self):
        """
        Trả về số lượng các transition đã được lưu trong bộ nhớ.
        Hàm len() sẽ gọi phương thức này.
        """
        # Chúng ta có thể trả về độ dài của bất kỳ list nào, vì chúng luôn bằng nhau.
        return len(self.states)
    def get_batch(self):
        """
        (Tùy chọn) Một hàm để lấy toàn bộ dữ liệu dưới dạng mảng NumPy.
        Hàm update của PPOTrainer đang truy cập trực tiếp nên hàm này không cần thiết,
        nhưng có thể hữu ích cho các mục đích khác.
        """
        return (
            np.array(self.states),
            np.array(self.actions),
            np.array(self.rewards),
            np.array(self.dones),
            np.array(self.log_probs)
        )