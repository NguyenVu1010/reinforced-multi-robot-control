#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import threading
from collections import deque

# --- THÊM CÁC IMPORT CẦN THIẾT ---
from sensor_msgs.msg import LaserScan
# Bỏ 'tf' và 'quaternion_matrix'
from tf.transformations import euler_from_quaternion # Chỉ cần hàm này

# Import các kiểu message cũ
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from matplotlib.image import imread
from matplotlib.widgets import Button
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from tf.transformations import quaternion_from_euler

# --- HẰNG SỐ TỐI ƯU HÓA ---
TRAJ_MAX_LEN = 500
TARGET_FRAME = "map" 

class MultiRobotPlotter:
    def __init__(self, robot_names, background_image_path=None, bg_extent=None):
        self.robot_names = robot_names
        self.num_robots = len(robot_names)
        
        # --- BỎ TF LISTENER ---
        # self.tf_listener = tf.TransformListener()
        
        # --- CÁC CẤU TRÚC DỮ LIỆU ---
        self.paths = {name: {'x': [], 'y': []} for name in robot_names}
        self.trajectories = {name: {'x': deque(maxlen=TRAJ_MAX_LEN), 'y': deque(maxlen=TRAJ_MAX_LEN)} for name in robot_names}
        # --- THÊM DỮ LIỆU GÓC CHO ROBOT ---
        self.current_poses = {name: {'x': 0, 'y': 0, 'theta': 0} for name in robot_names}
        self.future_points = {name: {'x': [], 'y': []} for name in robot_names}
        self.lidar_points = {name: {'x': [], 'y': []} for name in robot_names}
        
        self.lock = threading.Lock()

        # ... Phần cấu hình Matplotlib không thay đổi ...
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        if background_image_path and bg_extent:
            try:
                img = imread(background_image_path)
                self.ax.imshow(img, extent=bg_extent, origin='upper', alpha=0.5)
                rospy.loginfo(f"Hiển thị ảnh nền từ {background_image_path}")
                self.ax.set_xlim(bg_extent[0], bg_extent[1])
                self.ax.set_ylim(bg_extent[2], bg_extent[3])
            except Exception as e:
                rospy.logwarn(f"Lỗi khi tải ảnh nền: {e}")

        self.lines = {}
        colors = plt.cm.jet(np.linspace(0, 1, self.num_robots))
        for i, name in enumerate(self.robot_names):
            path_line, = self.ax.plot([], [], '--', color=colors[i], label=f'{name} Path')
            traj_line, = self.ax.plot([], [], '-', color=colors[i], linewidth=2, label=f'{name} Traj.')
            marker, = self.ax.plot([], [], 'o', color=colors[i], markersize=10, label=f'{name}')
            future_points_line, = self.ax.plot([], [], 'D', color=colors[i], markersize=7, label=f'{name} FPs' if i == 0 else "")
            lidar_scatter, = self.ax.plot([], [], ',', color=colors[i], alpha=0.7, label=f'{name} LiDAR' if i == 0 else "")

            self.lines[name] = {
                'path': path_line, 'traj': traj_line, 'marker': marker, 
                'future_points': future_points_line, 'lidar': lidar_scatter
            }

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_title("Real-Time Multi-Robot Path Following ")
        self.ax.legend(fontsize='small', ncol=2)
        self.ax.grid(True)
        self.ax.set_aspect('equal', adjustable='box')
        ax_reset_button = self.fig.add_axes([0.4, 0.01, 0.2, 0.04])
        self.reset_button = Button(ax_reset_button, 'Reset Models', color='lightcoral', hovercolor='0.9')
        self.reset_button.on_clicked(lambda event: self.reset_all_models())

    def path_callback(self, msg, robot_name):
        with self.lock:
            if robot_name not in self.paths: return
            self.paths[robot_name]['x'] = [p.pose.position.x for p in msg.poses]
            self.paths[robot_name]['y'] = [p.pose.position.y for p in msg.poses]
            self.trajectories[robot_name]['x'].clear()
            self.trajectories[robot_name]['y'].clear()
            if self.paths[robot_name]['x']:
                start_x = self.paths[robot_name]['x'][0]
                start_y = self.paths[robot_name]['y'][0]
                self.trajectories[robot_name]['x'].append(start_x)
                self.trajectories[robot_name]['y'].append(start_y)

    # --- SỬA CALLBACK NÀY ĐỂ LẤY THÊM GÓC ---
    def model_states_callback(self, msg):
        """Callback cho /gazebo/model_states, lấy vị trí và góc."""
        with self.lock:
            for name in self.robot_names:
                try:
                    idx = msg.name.index(name)
                    pose = msg.pose[idx]
                    
                    # Lấy vị trí x, y
                    x, y = pose.position.x, pose.position.y
                    
                    # Lấy orientation dạng quaternion
                    orientation_q = pose.orientation
                    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                    
                    # Chuyển đổi quaternion sang góc Euler
                    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    
                    # Cập nhật pose của robot, bao gồm cả góc yaw (theta)
                    self.current_poses[name]['x'] = x
                    self.current_poses[name]['y'] = y
                    self.current_poses[name]['theta'] = yaw # Lưu góc yaw
                    
                    # Cập nhật quỹ đạo
                    self.trajectories[name]['x'].append(x)
                    self.trajectories[name]['y'].append(y)
                except ValueError:
                    continue
    
    def future_points_marker_callback(self, msg, robot_name):
        with self.lock:
            if robot_name in self.future_points:
                self.future_points[robot_name]['x'] = [p.x for p in msg.points]
                self.future_points[robot_name]['y'] = [p.y for p in msg.points]

    # --- CALLBACK LIDAR ĐƯỢC SỬA LẠI HOÀN TOÀN ---
    def lidar_callback(self, msg, robot_name):
        """
        Callback xử lý dữ liệu LaserScan, chuyển đổi thủ công và lưu trữ các điểm.
        """
        # Lấy pose hiện tại của robot (vị trí + góc) một cách an toàn
        with self.lock:
            # Sao chép để tránh race condition
            robot_pose = self.current_poses[robot_name].copy() 

        # Nếu chưa có pose, bỏ qua
        if robot_pose is None:
            return

        robot_x = robot_pose['x']
        robot_y = robot_pose['y']
        robot_theta = robot_pose['theta']
        
        # Tạo mảng các góc cho mỗi tia laser
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        
        # Lọc ra các tia laser hợp lệ
        valid_indices = np.isfinite(msg.ranges)
        ranges = np.array(msg.ranges)[valid_indices]
        angles = angles[valid_indices]

        # Chuyển từ tọa độ cực (khoảng cách, góc) sang tọa độ Descartes (x, y)
        # trong hệ quy chiếu của laser
        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)
        
        # --- PHÉP BIẾN ĐỔI AFFINE 2D ---
        # Đây là phần cốt lõi thay thế cho TF.
        # 1. Xoay các điểm laser theo góc của robot (robot_theta)
        # 2. Tịnh tiến các điểm đã xoay đến vị trí của robot (robot_x, robot_y)
        
        cos_theta = np.cos(robot_theta)
        sin_theta = np.sin(robot_theta)
        
        # Áp dụng phép quay và tịnh tiến
        world_x = (x_local * cos_theta - y_local * sin_theta) + robot_x
        world_y = (x_local * sin_theta + y_local * cos_theta) + robot_y
        
        # Cập nhật dữ liệu vào cấu trúc chung
        with self.lock:
            self.lidar_points[robot_name]['x'] = world_x
            self.lidar_points[robot_name]['y'] = world_y
            
    def update_plot(self):
        # Hàm này không thay đổi
        with self.lock:
            for name in self.robot_names:
                self.lines[name]['path'].set_data(self.paths[name]['x'], self.paths[name]['y'])
                self.lines[name]['traj'].set_data(self.trajectories[name]['x'], self.trajectories[name]['y'])
                self.lines[name]['marker'].set_data([self.current_poses[name]['x']], [self.current_poses[name]['y']])
                self.lines[name]['future_points'].set_data(self.future_points[name]['x'], self.future_points[name]['y'])
                self.lines[name]['lidar'].set_data(self.lidar_points[name]['x'], self.lidar_points[name]['y'])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def reset_all_models(self):
        # Hàm này không thay đổi
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        for i, name in enumerate(self.robot_names):
            state = ModelState()
            state.model_name = name
            state.pose.position.x = 0.0 + i * 6.0
            state.pose.position.y = -2.0
            quat = quaternion_from_euler(0, 0, 0.0)
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, state.pose.orientation.w = quat
            state.reference_frame = "world"
            try:
                resp = set_model_state(state)
                if resp.success: rospy.loginfo(f"[{name}] ✅ Reset model thành công.")
                else: rospy.logwarn(f"[{name}] ❌ Reset model thất bại: {resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"[{name}] 🚨 Lỗi khi gọi reset model: {e}")

def main():
    # Hàm này không thay đổi
    try:
        rospy.init_node('multi_robot_plotter_node', anonymous=True)
        
        robot_names = ['robot1', 'robot2', 'robot3']  
        bg_path = "/home/nguyen1/catkin_ws/src/mir_robot/mir_control/map_output.png"
        bg_extent = (-29.75, 29.75, -19.3, 19.3)
        plotter = MultiRobotPlotter(robot_names, background_image_path=bg_path, bg_extent=bg_extent)
        
        rospy.Subscriber('/gazebo/model_states', ModelStates, plotter.model_states_callback, queue_size=1)
        
        for name in robot_names:
            path_topic_name = f'/paths/{name}/planned_path'
            rospy.Subscriber(path_topic_name, Path, plotter.path_callback, callback_args=name)
            
            marker_topic_name = f'/debug/{name}/future_points_marker'
            rospy.Subscriber(marker_topic_name, Marker, plotter.future_points_marker_callback, callback_args=name)

            lidar_topic_name = f'/{name}/scan'
            rospy.Subscriber(lidar_topic_name, LaserScan, plotter.lidar_callback, callback_args=name, queue_size=1)
        
        rospy.loginfo("Multi-Robot Plotter with LiDAR (Manual TF) đang chạy. Đóng cửa sổ biểu đồ để dừng node.")
        rate = rospy.Rate(20)
        
        while not rospy.is_shutdown() and plt.fignum_exists(plotter.fig.number):
            plotter.update_plot()
            rate.sleep()
            
    except rospy.ROSInterruptException:
        pass
    finally:
        plt.ioff()
        plt.close('all')
        rospy.loginfo("Plotter đã tắt.")

if __name__ == '__main__':
    main()