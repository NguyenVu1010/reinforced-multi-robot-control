#!/usr/bin/env python3

import rospy
import traceback
import os
import rospkg

# Import các lớp quản lý từ package trainer_logic
from train import ConfigManager, ModelManager, TrainingOrchestrator, SystemController

def main():
    """
    Hàm chính để khởi tạo các thành phần và chạy quá trình huấn luyện.
    """
    try:
        rospy.init_node('unified_trainer_node', anonymous=True)

        # Lấy đường dẫn config từ rosparam hoặc giá trị mặc định
        rospack = rospkg.RosPack()
        default_pkg_name = 'mir_control' 
        default_config = os.path.join(rospack.get_path(default_pkg_name), 'config', 'config.yaml')
        config_path = rospy.get_param("~config_path", default_config)

        # 1. Khởi tạo các lớp quản lý
        rospy.loginfo("--- Bước 1: Khởi tạo các Manager ---")
        cfg_manager = ConfigManager(config_path)
        sys_controller = SystemController(cfg_manager.pause_file, cfg_manager.stop_file)
        model_manager = ModelManager(
            model_cfg=cfg_manager.model_cfg,
            env_cfg=cfg_manager.env_cfg,
            checkpoint_cfg=cfg_manager.checkpoint_cfg,
            save_dir=cfg_manager.save_dir
        )

        # 2. Khởi tạo bộ điều phối chính
        rospy.loginfo("--- Bước 2: Khởi tạo Training Orchestrator ---")
        orchestrator = TrainingOrchestrator(cfg_manager, model_manager, sys_controller)

        # 3. Chạy quá trình huấn luyện
        rospy.loginfo("--- Bước 3: Bắt đầu vòng lặp huấn luyện ---")
        orchestrator.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("Chương trình kết thúc bởi ROS (Ctrl+C).")
    except Exception as e:
        rospy.logfatal(f"Đã xảy ra lỗi nghiêm trọng không thể phục hồi: {e}")
        traceback.print_exc()

if __name__ == '__main__':
    main()