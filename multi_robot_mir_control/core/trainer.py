# core/trainer.py

import torch
import torch.nn as nn
import numpy as np
import rospy

class PPOTrainer:
    def __init__(self, policy_net, value_net, lr_actor, lr_critic, 
                 gamma, gae_lambda, clip_epsilon, k_epochs, entropy_coef):
        
        self.policy_net = policy_net
        self.value_net = value_net
        
        # Lấy thiết bị từ chính model đã được chuyển lên GPU
        self.device = next(self.policy_net.parameters()).device
        rospy.loginfo(f"[PPOTrainer] Sẽ thực hiện cập nhật trên thiết bị: {self.device}")

        # Khởi tạo các siêu tham số
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        self.clip_epsilon = clip_epsilon
        self.k_epochs = k_epochs
        self.entropy_coef = entropy_coef

        # Khởi tạo optimizers
        self.optimizer_policy = torch.optim.Adam(self.policy_net.parameters(), lr=lr_actor)
        self.optimizer_value = torch.optim.Adam(self.value_net.parameters(), lr=lr_critic)
        
        # Hàm loss cho value network
        self.mse_loss = nn.MSELoss()

    def calculate_ppo_loss(self, log_probs, old_log_probs, advantages):
        """
        Tính toán phần loss của policy (Actor loss) theo công thức PPO-Clip.
        """
        # Tính tỷ lệ (ratio): r_t(theta) = exp(log pi_theta(a_t|s_t) - log pi_theta_old(a_t|s_t))
        ratios = torch.exp(log_probs - old_log_probs.detach())

        # Tính toán hai thành phần của hàm mục tiêu surrogate
        surr1 = ratios * advantages
        surr2 = torch.clamp(ratios, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages

        # PPO loss là giá trị nhỏ nhất của hai thành phần này, lấy trung bình và đổi dấu
        policy_loss = -torch.min(surr1, surr2).mean()
        
        return policy_loss

    def update(self, memory):
        torch.autograd.set_detect_anomaly(True)  # Bật debug gradient

        # --- 1. Tính rewards-to-go ---
        rewards = []
        discounted_reward = 0
        for reward, done in zip(reversed(memory.rewards), reversed(memory.dones)):
            if done:
                discounted_reward = 0
            discounted_reward = reward + self.gamma * discounted_reward
            rewards.insert(0, discounted_reward)

        # --- 2. Chuyển dữ liệu lên GPU ---
        try:
            old_states     = torch.FloatTensor(np.array(memory.states)).to(self.device)
            old_actions    = torch.FloatTensor(np.array(memory.actions)).to(self.device)
            old_log_probs  = torch.FloatTensor(np.array(memory.log_probs)).to(self.device)
            rewards_to_go  = torch.FloatTensor(rewards).to(self.device)
        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi chuyển dữ liệu lên GPU: {e}")
            return

        # --- 3. Kiểm tra NaN ngay ---
        if torch.isnan(old_states).any():
            rospy.logerr("❌ old_states chứa NaN! Huỷ update.")
            return
        if torch.isnan(old_actions).any():
            rospy.logerr("❌ old_actions chứa NaN! Huỷ update.")
            return
        if torch.isnan(old_log_probs).any():
            rospy.logerr("❌ old_log_probs chứa NaN! Huỷ update.")
            return
        if torch.isnan(rewards_to_go).any():
            rospy.logerr("❌ rewards_to_go chứa NaN! Huỷ update.")
            return

        # --- 4. Tính advantage ---
        with torch.no_grad():
            old_state_values = self.value_net(old_states).squeeze()

        advantages = rewards_to_go - old_state_values
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        advantages = torch.clamp(advantages, -10, 10)  # Chống bùng

        rospy.loginfo(f"[PPOTrainer] Bắt đầu {self.k_epochs} epoch trên {len(old_states)} mẫu.")

        for epoch in range(self.k_epochs):
            # --- Policy ---
            log_probs, dist_entropy = self.policy_net.evaluate(old_states, old_actions)

            if torch.isnan(log_probs).any():
                rospy.logerr("❌ log_probs chứa NaN!")
                return

            policy_loss = self.calculate_ppo_loss(log_probs, old_log_probs, advantages)
            entropy_loss = -self.entropy_coef * dist_entropy.mean()
            total_policy_loss = policy_loss + entropy_loss

            if torch.isnan(total_policy_loss).any():
                rospy.logerr("❌ total_policy_loss chứa NaN! Huỷ update.")
                return

            self.optimizer_policy.zero_grad()
            total_policy_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), max_norm=0.5)
            self.optimizer_policy.step()

            # --- Value ---
            current_state_values = self.value_net(old_states).squeeze()
            value_loss = self.mse_loss(current_state_values, rewards_to_go)

            if torch.isnan(value_loss).any():
                rospy.logerr("❌ value_loss chứa NaN! Huỷ update.")
                return

            self.optimizer_value.zero_grad()
            value_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.value_net.parameters(), max_norm=0.5)
            self.optimizer_value.step()
