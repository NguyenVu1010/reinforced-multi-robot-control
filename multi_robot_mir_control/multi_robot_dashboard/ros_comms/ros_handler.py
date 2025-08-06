# ros_comms/ros_handler.py

import rospy
import threading
import time
import subprocess
from subprocess import TimeoutExpired
import sys
import os
import numpy as np
from queue import Queue, Empty

# Import các kiểu message cần thiết
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

# --- CẤU TRÚC DỮ LIỆU TOÀN CỤC ---
GLOBAL_DATA = {}
DATA_LOCK = threading.Lock()

class RosHandler:
    def __init__(self, launch_command, robot_names):
        global GLOBAL_DATA
        self.launch_command = launch_command
        self.robot_names = robot_names
        self.ros_process = None
        self.is_running = True
        self.command_queue = Queue()
        self.subscribers = []

        # Tạo bảng màu cho robot
        colors = [
            (31, 119, 180), (255, 127, 14), (44, 160, 44), 
            (214, 39, 40), (148, 103, 189), (140, 86, 75)
        ]
        # Đảm bảo có đủ màu, lặp lại nếu cần
        robot_colors = {name: f'rgb({colors[i % len(colors)][0]},{colors[i % len(colors)][1]},{colors[i % len(colors)][2]})' for i, name in enumerate(robot_names)}

        with DATA_LOCK:
            GLOBAL_DATA['status_message'] = "Kiểm tra kết nối ROS..."
            GLOBAL_DATA['ros_launched'] = False
            GLOBAL_DATA['ros_connected'] = False
            GLOBAL_DATA['poses'] = {name: {} for name in robot_names}
            GLOBAL_DATA['lidar'] = {name: [] for name in robot_names}
            GLOBAL_DATA['robot_colors'] = robot_colors

    def _execute_launch(self):
        """Khởi chạy file launch của ROS trong một tiến trình riêng."""
        if self.ros_process and self.ros_process.poll() is None:
            print("INFO: ROS process is already running.")
            with DATA_LOCK:
                GLOBAL_DATA['status_message'] = "ROS is already running."
            return

        try:
            # Lấy đường dẫn workspace một cách an toàn
            catkin_ws_path = os.path.expanduser("~/catkin_ws")
            full_command = (
                f"source /opt/ros/{os.environ.get('ROS_DISTRO', 'noetic')}/setup.bash && "
                f"source {catkin_ws_path}/devel/setup.bash && "
                f"{self.launch_command}"
            )
            print(f"INFO: Executing command:\n  {full_command}")
            
            with DATA_LOCK:
                GLOBAL_DATA['status_message'] = "Launching ROS... Please wait."

            self.ros_process = subprocess.Popen(
                full_command, shell=True, executable='/bin/bash',
                stdout=sys.stdout, stderr=sys.stderr, preexec_fn=os.setsid
            )
            with DATA_LOCK:
                GLOBAL_DATA['ros_launched'] = True
                GLOBAL_DATA['status_message'] = "ROS launched. Waiting for connection..."
        except Exception as e:
            with DATA_LOCK:
                GLOBAL_DATA['ros_launched'] = False
                GLOBAL_DATA['status_message'] = f"Launch failed: {e}"
            print(f"ERROR: Failed to launch ROS. {e}")

    def main_loop(self):
        """
        Vòng lặp chính, chạy trong luồng riêng.
        Tự động kiểm tra kết nối và xử lý lệnh từ giao diện.
        """
        print("INFO: ROS Handler thread started. Immediately checking for ROS Master...")
        is_node_initialized = False

        while self.is_running:
            # 1. Xử lý các lệnh từ giao diện (không chặn)
            try:
                command = self.command_queue.get_nowait()
                if command == 'LAUNCH':
                    self._execute_launch()
            except Empty:
                pass

            # 2. Kiểm tra kết nối ROS
            try:
                rospy.get_master().getSystemState()
                
                with DATA_LOCK:
                    if not GLOBAL_DATA.get('ros_connected', False):
                        print("\nSUCCESS: Connected to ROS Master!\n")
                        GLOBAL_DATA['ros_connected'] = True
                        GLOBAL_DATA['ros_launched'] = True
                        GLOBAL_DATA['status_message'] = "Connection successful!"
                        
                        if not is_node_initialized:
                            try:
                                rospy.init_node('dashboard_connector_node', anonymous=True, disable_signals=True)
                                is_node_initialized = True
                                print("INFO: ROS thread initialized node.")
                            except rospy.ROSInitException as e:
                                print(f"ERROR: Failed to initialize ROS node: {e}")
                        
                        if is_node_initialized:
                            self.register_subscribers()
            
            except Exception:
                with DATA_LOCK:
                    if GLOBAL_DATA.get('ros_connected', False):
                        print("WARN: Connection to ROS Master lost. Attempting to reconnect...")
                        for sub in self.subscribers:
                            sub.unregister()
                        self.subscribers = []
                        
                    GLOBAL_DATA['ros_connected'] = False
                    if not GLOBAL_DATA.get('ros_launched', False):
                         GLOBAL_DATA['status_message'] = "ROS Master not found. Please launch ROS."
            
            time.sleep(1)
        
        print("INFO: RosHandler thread has exited.")

    def shutdown(self):
        """Dọn dẹp tài nguyên khi ứng dụng tắt."""
        print("INFO: Shutting down RosHandler...")
        self.is_running = False
        if self.ros_process:
            import signal
            try:
                pgid = os.getpgid(self.ros_process.pid)
                os.killpg(pgid, signal.SIGTERM)
                self.ros_process.wait(timeout=5)
                print("INFO: ROS process terminated.")
            except (ProcessLookupError, TimeoutExpired):
                 os.killpg(pgid, signal.SIGKILL)
                 print("WARN: ROS process did not terminate gracefully. Forced kill.")
            except Exception as e:
                print(f"ERROR during ROS process shutdown: {e}")
        
        if rospy.is_initialized():
            rospy.signal_shutdown("Dashboard is shutting down.")
            
        print("INFO: RosHandler shutdown complete.")

    def register_subscribers(self):
        """Đăng ký tất cả các subscriber cần thiết sau khi đã kết nối."""
        if self.subscribers:
            print("INFO: Subscribers already registered.")
            return
            
        rospy.loginfo("Registering ROS subscribers...")
        
        sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.subscribers.append(sub)
        
        for name in self.robot_names:
            topic_name = f'/{name}/scan'
            sub = rospy.Subscriber(topic_name, LaserScan, self.lidar_callback, callback_args=name, queue_size=1)
            self.subscribers.append(sub)
        
        rospy.loginfo(f"Successfully registered {len(self.subscribers)} subscribers.")

    def model_states_callback(self, msg):
        """Callback cho /gazebo/model_states, cập nhật vị trí và góc."""
        with DATA_LOCK:
            if not GLOBAL_DATA.get('ros_connected', False): return

            for name in self.robot_names:
                try:
                    idx = msg.name.index(name)
                    pose = msg.pose[idx]
                    x, y = pose.position.x, pose.position.y
                    q = pose.orientation
                    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
                    GLOBAL_DATA['poses'][name] = {'x': x, 'y': y, 'theta': yaw}
                except ValueError:
                    continue
    
    def lidar_callback(self, msg, robot_name):
        """Callback xử lý dữ liệu LaserScan, chuyển đổi và lưu trữ các điểm."""
        with DATA_LOCK:
            if not GLOBAL_DATA.get('ros_connected', False): return
            pose = GLOBAL_DATA['poses'].get(robot_name, {})
        
        if not pose: return

        robot_x, robot_y, robot_theta = pose['x'], pose['y'], pose['theta']
        
        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        valid_indices = np.isfinite(msg.ranges)
        ranges = np.array(msg.ranges)[valid_indices]
        angles = angles[valid_indices]

        x_local = ranges * np.cos(angles)
        y_local = ranges * np.sin(angles)
        
        cos_theta, sin_theta = np.cos(robot_theta), np.sin(robot_theta)
        
        world_x = (x_local * cos_theta - y_local * sin_theta) + robot_x
        world_y = (x_local * sin_theta + y_local * cos_theta) + robot_y
        
        lidar_points = list(zip(world_x, world_y))

        with DATA_LOCK:
            GLOBAL_DATA['lidar'][robot_name] = lidar_points