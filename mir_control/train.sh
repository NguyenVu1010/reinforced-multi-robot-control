#!/bin/bash
echo "🔧 Đang biên dịch workspace ROS..."

cd ~/catkin_ws
catkin_make
source devel/setup.bash
echo "✅ Đã source xong môi trường catkin_ws."
echo "🚀 Đang khởi động hệ thống huấn luyện PPO với ROS..."

# ✅ 1. Cấu hình GPU
export CUDA_VISIBLE_DEVICES=0
unset TORCH_CUDA_ALLOC_CONF

# ✅ 2. Tạo thư mục log nếu chưa có
mkdir -p logs
log_file="logs/train_$(date +%F_%H-%M-%S).log"

# ✅ 3. Khởi động mô phỏng Gazebo (file launch 1)
echo "▶️ Launching Gazebo..." | tee -a "$log_file"
roslaunch mir_gazebo multi_robot_simulation.launch gui:=false &
gazebo_pid=$!

# ✅ 4. Đợi Gazebo khởi động ổn định
sleep 10

# ✅ 5. Bắt đầu vòng lặp huấn luyện (file launch 2)
while true
do
    start_time=$(date +%s)

    echo "▶️ Bắt đầu huấn luyện PPO..." | tee -a "$log_file"
    roslaunch mir_control train.launch >> "$log_file" 2>&1

    if [ $? -eq 0 ]; then
        echo "✅ Huấn luyện hoàn tất!" | tee -a "$log_file"
        break
    else
        echo "⚠️ train.launch bị lỗi. Sẽ thử lại sau 5 giây..." | tee -a "$log_file"
        sleep 5
    fi

    end_time=$(date +%s)
    duration=$((end_time - start_time))
    echo "⏱️ Thời gian chạy: ${duration}s" | tee -a "$log_file"
done

# ✅ 6. Dừng mô phỏng khi xong
kill $gazebo_pid
echo "🧹 Đã dừng Gazebo." | tee -a "$log_file"
