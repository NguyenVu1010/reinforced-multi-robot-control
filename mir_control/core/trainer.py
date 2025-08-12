# core/trainer.py

import torch
import torch.nn as nn
import numpy as np
import rospy
from torch.amp import GradScaler, autocast

# =========================================================================
# --- Tối ưu hóa hàm tính GAE bằng torch.jit.script ---
# Đặt hàm này bên ngoài lớp để nó là một hàm độc lập
@torch.jit.script
def compute_gae_and_returns(rewards, dones, values, gamma: float, gae_lambda: float):
    """
    Tính toán Generalized Advantage Estimation (GAE) và returns.
    Hàm này được JIT-compiled để tăng tốc độ.
    """
    advantages = torch.zeros_like(rewards)
    last_gae_lam = torch.tensor(0.0, device=rewards.device)
    
    # Chuyển dones sang float
    dones_float = dones.float()

    for t in range(len(rewards) - 1, -1, -1): # Lặp ngược
        # Lấy giá trị của state tiếp theo, bằng 0 nếu là terminal
        next_non_terminal = 1.0 - dones_float[t]
        next_value = values[t + 1] if t < len(rewards) - 1 else torch.tensor(0.0, device=rewards.device)
        
        # Tính TD-error (delta)
        delta = rewards[t] + gamma * next_value * next_non_terminal - values[t]
        
        # Công thức GAE
        advantages[t] = last_gae_lam = delta + gamma * gae_lambda * next_non_terminal * last_gae_lam
    
    # Tính returns từ advantages
    returns = advantages + values
    return advantages, returns
# =========================================================================


class PPOTrainer:
    def __init__(self, policy_net, value_net, lr_actor, lr_critic, 
                 gamma, gae_lambda, clip_epsilon, k_epochs, entropy_coef,
                 mini_batch_size):
        
        self.policy_net = policy_net
        self.value_net = value_net
        
        self.device = next(self.policy_net.parameters()).device
        rospy.loginfo(f"[PPOTrainer] Sẽ thực hiện cập nhật trên thiết bị: {self.device}")

        # Khởi tạo các siêu tham số
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.k_epochs = k_epochs
        self.entropy_coef = entropy_coef
        self.mini_batch_size = mini_batch_size

        # Khởi tạo optimizers
        self.optimizer_policy = torch.optim.Adam(self.policy_net.parameters(), lr=lr_actor)
        self.optimizer_value = torch.optim.Adam(self.value_net.parameters(), lr=lr_critic)
        
        self.mse_loss = nn.MSELoss()

        # Khởi tạo GradScaler cho Mixed Precision
        self.device_type = self.device.type
        self.use_amp = self.device_type == 'cuda'
        self.scaler = GradScaler(enabled=self.use_amp)
        if self.use_amp:
            rospy.loginfo("[PPOTrainer] Automatic Mixed Precision (AMP) đã được bật.")

    def calculate_ppo_loss(self, log_probs, old_log_probs, advantages):
        """Tính toán phần loss của policy (Actor loss) theo công thức PPO-Clip."""
        ratios = torch.exp(log_probs - old_log_probs.detach())
        surr1 = ratios * advantages
        surr2 = torch.clamp(ratios, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages
        policy_loss = -torch.min(surr1, surr2).mean()
        return policy_loss

    def update(self, memory):
        # --- 1. Chuyển dữ liệu sang Tensor ---
        try:
            old_states     = torch.FloatTensor(np.array(memory.states)).to(self.device)
            old_actions    = torch.FloatTensor(np.array(memory.actions)).to(self.device)
            old_log_probs  = torch.FloatTensor(np.array(memory.log_probs)).to(self.device)
            rewards        = torch.FloatTensor(np.array(memory.rewards)).to(self.device)
            dones          = torch.BoolTensor(np.array(memory.dones)).to(self.device)
        except Exception as e:
            rospy.logerr(f"❌ [Trainer] Lỗi khi chuyển dữ liệu memory sang Tensor: {e}")
            return -1, -1, -1

        # --- 2. Tính toán State Values ---
        with torch.no_grad():
            old_state_values = self.value_net(old_states).squeeze()
        
        # =========================================================================
        # --- 3. GỌI HÀM ĐÃ ĐƯỢC JIT-COMPILED ĐỂ TÍNH GAE VÀ RETURNS ---
        with torch.no_grad():
            advantages, returns = compute_gae_and_returns(
                rewards,
                dones,
                old_state_values,
                self.gamma,
                self.gae_lambda
            )
        # =========================================================================
        
        # --- 4. Chuẩn hóa Advantage ---
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        advantages = torch.clamp(advantages, -10, 10)

        # --- 5. Vòng lặp tối ưu hóa với Mini-batching và AMP ---
        rospy.loginfo(f"[PPOTrainer] Bắt đầu {self.k_epochs} epoch trên {len(old_states)} mẫu, mini-batch size: {self.mini_batch_size}.")
        
        batch_size = old_states.size(0)
        final_policy_loss, final_value_loss, final_entropy = 0, 0, 0

        for epoch in range(self.k_epochs):
            indices = torch.randperm(batch_size)
            
            for start in range(0, batch_size, self.mini_batch_size):
                end = start + self.mini_batch_size
                mini_batch_indices = indices[start:end]

                mb_states = old_states[mini_batch_indices]
                mb_actions = old_actions[mini_batch_indices]
                mb_old_log_probs = old_log_probs[mini_batch_indices]
                mb_advantages = advantages[mini_batch_indices]
                mb_returns = returns[mini_batch_indices]
                
                with autocast(device_type=self.device_type, enabled=self.use_amp):
                    log_probs, dist_entropy = self.policy_net.evaluate(mb_states, mb_actions)
                    policy_loss = self.calculate_ppo_loss(log_probs, mb_old_log_probs, mb_advantages)
                    entropy_loss = -self.entropy_coef * dist_entropy.mean()
                    total_policy_loss = policy_loss + entropy_loss

                    current_state_values = self.value_net(mb_states).squeeze()
                    value_loss = self.mse_loss(current_state_values, mb_returns)

                # Tối ưu hóa với GradScaler
                self.optimizer_policy.zero_grad(set_to_none=True)
                self.scaler.scale(total_policy_loss).backward()
                self.scaler.unscale_(self.optimizer_policy)
                torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=0.5)
                self.scaler.step(self.optimizer_policy)

                self.optimizer_value.zero_grad(set_to_none=True)
                self.scaler.scale(value_loss).backward()
                self.scaler.unscale_(self.optimizer_value)
                torch.nn.utils.clip_grad_norm_(self.value_net.parameters(), max_norm=0.5)
                self.scaler.step(self.optimizer_value)

                self.scaler.update()

                if epoch == self.k_epochs - 1:
                    final_policy_loss = total_policy_loss.item()
                    final_value_loss = value_loss.item()
                    final_entropy = dist_entropy.mean().item()

        # --- 6. Trả về các giá trị loss để logging ---
        return final_policy_loss, final_value_loss, final_entropy