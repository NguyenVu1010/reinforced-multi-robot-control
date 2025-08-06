# utils/logger.py

import torch
import os

def setup_save_dir(save_dir="saved_models"):
    """
    Tạo thư mục để lưu trữ nếu nó chưa tồn tại.
    Trả về đường dẫn thư mục.
    """
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    return save_dir

def save_model(model, save_dir, filename):
    """
    Lưu state_dict của mô hình vào một file .pth.
    
    Args:
        model (torch.nn.Module): Mô hình cần lưu.
        save_dir (str): Thư mục để lưu.
        filename (str): Tên file (ví dụ: 'policy_net.pth').
    """
    if not os.path.exists(save_dir):
        print(f"Thư mục {save_dir} không tồn tại. Đang tạo...")
        os.makedirs(save_dir)
        
    filepath = os.path.join(save_dir, filename)
    torch.save(model.state_dict(), filepath)
    print(f"Đã lưu mô hình tại: {filepath}")

def log_weights_to_text(model, save_dir, filename):
    """
    Ghi tất cả các trọng số và bias của model ra file text.
    
    Args:
        model (torch.nn.Module): Mô hình cần log.
        save_dir (str): Thư mục để lưu.
        filename (str): Tên file (ví dụ: 'weights.txt').
    """
    if not os.path.exists(save_dir):
        print(f"Thư mục {save_dir} không tồn tại. Đang tạo...")
        os.makedirs(save_dir)
        
    filepath = os.path.join(save_dir, filename)
    with open(filepath, 'w') as f:
        f.write(f"Model: {model.__class__.__name__}\n")
        f.write("="*40 + "\n")
        
        for name, param in model.named_parameters():
            if param.requires_grad:
                f.write(f"Layer: {name}\n")
                f.write(f"Shape: {param.shape}\n")
                param_data = param.data.cpu().numpy()
                f.write(f"Values:\n{param_data}\n")
                f.write("-"*40 + "\n")
    print(f"Đã log trọng số ra file: {filepath}")