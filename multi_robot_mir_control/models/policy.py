# models/policy.py

import torch
import torch.nn as nn
from torch.distributions import Normal # <-- THAY ĐỔI: Dùng Normal thay vì MultivariateNormal

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, hidden_dim):
        super(PolicyNetwork, self).__init__()

        self.actor = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.Tanh(),
            nn.Linear(hidden_dim, action_dim)
        )
        
        # Học log của độ lệch chuẩn cho từng chiều action
        self.action_log_std = nn.Parameter(torch.zeros(action_dim)) # Shape (action_dim,) là đủ

    def forward(self, state):
        """
        Hàm forward trả về một đối tượng phân phối xác suất hành động.
        """
        action_mean = self.actor(state) # Shape (batch_size, action_dim)
        
        action_std = torch.exp(self.action_log_std) # Shape (action_dim,)
        
        # Tạo phân phối chuẩn. PyTorch sẽ tự động broadcast `action_std`
        # để khớp với batch_size của `action_mean`.
        # `mean` có shape (batch, action_dim), `std` có shape (action_dim) -> OK
        dist = Normal(action_mean, action_std)
        
        return dist

    def get_action(self, state):
        """
        Lấy một hành động (lấy mẫu) và log_prob của nó.
        """
        with torch.no_grad():
            dist = self.forward(state)
            action = dist.sample()
            
            # Khi các chiều độc lập, log_prob tổng hợp là tổng của các log_prob thành phần.
            # .sum(axis=-1) sẽ tính tổng theo chiều action.
            # Input: (batch, action_dim) -> Output: (batch,)
            log_prob = dist.log_prob(action).sum(axis=-1)
            
        return action, log_prob

    def evaluate(self, state, action):
        """
        Đánh giá một cặp (state, action) đã biết.
        """
        dist = self.forward(state)
        
        # Tính log-probability của các action.
        # .sum(axis=-1) để có được một giá trị log-prob cho mỗi transition trong batch.
        # Input: (batch, action_dim) -> Output: (batch,)
        log_prob = dist.log_prob(action).sum(axis=-1)
        
        # Tính entropy.
        # .sum(axis=-1) để có được entropy tổng hợp cho mỗi transition.
        # Input: (batch, action_dim) -> Output: (batch,)
        dist_entropy = dist.entropy().sum(axis=-1)
        
        # Trả về các tensor có shape (batch_size,)
        return log_prob, dist_entropy