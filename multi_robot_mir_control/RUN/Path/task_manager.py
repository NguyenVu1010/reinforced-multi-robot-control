#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from geometry_msgs.msg import Point
import rospkg

import threading
import json
import os
from datetime import datetime
import math
import traceback
from typing import List, Dict, Any, Tuple

# --- Hằng số cấu hình ---
MAX_RETRIES = 3
PICKUP_WAIT_TIME = 5.0
TASK_EXECUTION_TIMEOUT = 120.0
MIN_DISTANCE_THRESHOLD = 0.5  # (mét) Nếu khoảng cách di chuyển nhỏ hơn giá trị này, coi như đã đến nơi

class TaskManager:
    def __init__(self, robot_names: List[str], data_dir: str):
        self.robot_names = robot_names
        self.task_file = os.path.join(data_dir, 'tasks.json')
        self.robot_status_file = os.path.join(data_dir, 'robot_status.json')
        
        # SỬA LỖI DEADLOCK: Sử dụng Lock thông thường (không phải RLock)
        # và tái cấu trúc code để tránh gọi lock lồng nhau.
        self.lock = threading.Lock()
        
        self.worker_threads: Dict[str, threading.Thread | None] = {name: None for name in self.robot_names}
        rospy.loginfo(f"Task file path: {self.task_file}")
        rospy.loginfo(f"Robot status file path: {self.robot_status_file}")
        self.robot_status: Dict[str, Dict[str, Any]] = {}
        self._reset_all_statuses_on_startup()
        self._initialize_ros_communications()

    def _reset_all_statuses_on_startup(self):
        rospy.loginfo("Performing clean start: Resetting statuses...")
        for name in self.robot_names:
            self.robot_status[name] = {'status': 'idle', 'task_id': None, 'current_location': None}
        self.save_robot_status()
        
        with self.lock: # Lock để đảm bảo thao tác đọc-sửa-ghi là nguyên tử
            all_tasks = self._read_json_file(self.task_file)
            tasks_reset_count = 0
            for task in all_tasks:
                if task.get('status') in ['ACTIVE', 'ASSIGNED']:
                    task['status'] = 'PENDING'
                    task['assigned_to'] = None
                    task['current_stage'] = 'RESET_ON_STARTUP'
                    task['retries'] = 0
                    tasks_reset_count += 1
            if tasks_reset_count > 0:
                self._write_json_file(self.task_file, all_tasks)
                rospy.loginfo(f"Reset {tasks_reset_count} stale tasks to 'PENDING'.")

    def _initialize_ros_communications(self):
        self.path_publishers = {name: rospy.Publisher(f'/paths/{name}/planned_path', Path, queue_size=1, latch=True) for name in self.robot_names}
        self.completion_sub = rospy.Subscriber('/task_completion', String, self.completion_callback, queue_size=10)
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)
        self.service_name = 'request_path_service'
        try:
            from mir_control.srv import RequestPath, RequestPathRequest
            self.RequestPathRequest = RequestPathRequest
            rospy.wait_for_service(self.service_name, timeout=15.0)
            self.path_provider_client = rospy.ServiceProxy(self.service_name, RequestPath)
            rospy.loginfo(f"Đã kết nối thành công tới service '{self.service_name}'.")
        except Exception as e:
            rospy.logfatal(f"Không thể thiết lập giao tiếp ROS: {e}")
            rospy.signal_shutdown("Lỗi khởi tạo ROS communications.")

    # SỬA LỖI DEADLOCK: Hàm này không tự quản lý lock nữa
    def _read_json_file(self, filepath: str, default_type=list) -> Any:
        try:
            if not os.path.exists(filepath): return default_type()
            with open(filepath, 'r', encoding='utf-8') as f: return json.load(f)
        except (json.JSONDecodeError, FileNotFoundError): return default_type()

    # SỬA LỖI DEADLOCK: Hàm này không tự quản lý lock nữa
    def _write_json_file(self, filepath: str, data: Any):
        try:
            with open(filepath, 'w', encoding='utf-8') as f: json.dump(data, f, indent=2, ensure_ascii=False)
        except Exception as e: rospy.logerr(f"Không thể ghi file {filepath}: {e}")

    def save_robot_status(self):
        with self.lock: # Lấy lock để đọc trạng thái và ghi file
            status_to_save = {}
            for name, data in self.robot_status.items():
                saved_data = {k: v for k, v in data.items() if k != 'current_location'}
                saved_data['last_updated'] = datetime.utcnow().isoformat() + "Z"
                status_to_save[name] = saved_data
            self._write_json_file(self.robot_status_file, status_to_save)

    def update_task_in_file(self, task_id: str, updates: dict):
        with self.lock: # Lấy lock để đảm bảo chuỗi đọc-sửa-ghi là nguyên tử
            all_tasks = self._read_json_file(self.task_file)
            for task in all_tasks:
                if task.get('id') == task_id:
                    task.update(updates)
                    task['last_updated'] = datetime.utcnow().isoformat() + "Z"
                    self._write_json_file(self.task_file, all_tasks)
                    return
    
    def model_states_callback(self, msg: ModelStates):
        with self.lock:
            try:
                model_map = {name: pose.position for name, pose in zip(msg.name, msg.pose)}
                for robot_name in self.robot_names:
                    if robot_name in self.robot_status and robot_name in model_map:
                        self.robot_status[robot_name]['current_location'] = model_map[robot_name]
            except Exception: pass

    # SỬA LỖI DEADLOCK: Tách biệt việc cập nhật bộ nhớ và ghi file
    def completion_callback(self, msg: String):
        robot_name = msg.data
        
        # Bước 1: Chỉ cập nhật trạng thái trong bộ nhớ (nhanh, cần lock)
        with self.lock:
            robot_state = self.robot_status.get(robot_name)
            if not robot_state or not robot_state.get('task_id'): 
                return
            robot_state['status'] = 'stage_completed'
        
        # Bước 2: Lưu trạng thái ra file (chậm hơn, không nằm trong lock của callback)
        # Hàm save_robot_status sẽ tự quản lý lock của riêng nó.
        self.save_robot_status()
        rospy.loginfo(f"[Completion] Robot '{robot_name}' báo cáo. Trạng thái -> 'stage_completed'.")


    ### =============================================================== ###
    ###        LOGIC THỰC THI NHIỆM VỤ ĐÃ ĐƯỢC BỌC TRONG TRY...EXCEPT    ###
    ### =============================================================== ###
    def _execute_task_thread(self, task: Dict, robot_name: str):
        task_id = task['id']
        rospy.loginfo(f"[Thread-{robot_name}] Bắt đầu thực thi task '{task_id}'.")
        
        try:
            task_type = task.get('type')
            
            # Giai đoạn 1
            rospy.loginfo(f"[Thread-{robot_name}] Bắt đầu giai đoạn 1.")
            target_loc_1 = task.get('pickup_location') if task_type == 'DELIVERY' else task.get('target_location')
            if not self._execute_stage(task, robot_name, target_loc_1, stage_name='STAGE_1'):
                return # handle_task_failure đã được gọi bên trong _execute_stage

            # Giai đoạn 2 (Nếu là DELIVERY)
            if task_type == 'DELIVERY':
                rospy.loginfo(f"[Thread-{robot_name}] Giai đoạn 1 hoàn thành. Chờ {PICKUP_WAIT_TIME}s.")
                self.update_robot_and_task_status(robot_name, task_id, 'waiting_at_pickup')
                rospy.sleep(PICKUP_WAIT_TIME)
                
                rospy.loginfo(f"[Thread-{robot_name}] Bắt đầu giai đoạn 2.")
                target_loc_2 = task.get('dropoff_location')
                if not self._execute_stage(task, robot_name, target_loc_2, stage_name='STAGE_2'):
                    return

            # Hoàn thành thành công
            rospy.loginfo(f"--- [TASK COMPLETED] Task '{task_id}' bởi '{robot_name}'. ---")
            self.update_robot_and_task_status(robot_name, task_id, 'idle', task_status='COMPLETED')
        
        except Exception as e:
            rospy.logerr(f"[Thread-{robot_name}] Gặp lỗi nghiêm trọng không lường trước khi thực thi task '{task_id}'.")
            rospy.logerr(traceback.format_exc())
            self.handle_task_failure(task_id, robot_name, "UNHANDLED_EXCEPTION")

        finally:
            with self.lock:
                self.worker_threads[robot_name] = None
            rospy.loginfo(f"[Thread-{robot_name}] Luồng thực thi đã kết thúc.")

    def _execute_stage(self, task: Dict, robot_name: str, target_location: Dict, stage_name: str) -> bool:
        task_id, task_type = task['id'], task.get('type')
        if not target_location:
            self.handle_task_failure(task_id, robot_name, f"MISSING_LOCATION_{stage_name}")
            return False

        # TÍNH NĂNG MỚI: Kiểm tra khoảng cách trước khi di chuyển
        with self.lock:
            current_pos = self.robot_status[robot_name].get('current_location')
        
        if current_pos:
            target_p = Point(x=target_location['x'], y=target_location['y'], z=0.0)
            distance = math.hypot(current_pos.x - target_p.x, current_pos.y - target_p.y)
            if distance < MIN_DISTANCE_THRESHOLD:
                rospy.loginfo(f"[Thread-{robot_name}] Khoảng cách đến mục tiêu ({distance:.2f}m) quá ngắn. Bỏ qua di chuyển cho giai đoạn '{stage_name}'.")
                return True # Coi như giai đoạn đã hoàn thành thành công

        robot_status_str = {'STAGE_1': 'moving_to_pickup' if task_type == 'DELIVERY' else 'moving_to_target', 'STAGE_2': 'moving_to_dropoff'}.get(stage_name, 'moving')
        self.update_robot_and_task_status(robot_name, task_id, robot_status_str)
        if not self.send_move_command(robot_name, target_location):
            self.handle_task_failure(task_id, robot_name, f"PLANNING_FAILED_{stage_name}")
            return False
        if not self._wait_for_stage_completion(robot_name, task_id, timeout=TASK_EXECUTION_TIMEOUT):
            self.handle_task_failure(task_id, robot_name, f"TIMEOUT_{stage_name}")
            return False
        rospy.loginfo(f"[Thread-{robot_name}] Hoàn thành giai đoạn '{stage_name}' cho task '{task_id}'.")
        return True

    def update_robot_and_task_status(self, robot_name: str, task_id: str, robot_status: str, task_status: str = 'ACTIVE'):
        with self.lock:
            self.robot_status[robot_name]['status'] = robot_status
            self.robot_status[robot_name]['task_id'] = task_id if robot_status != 'idle' else None
        self.update_task_in_file(task_id, {'status': task_status, 'current_stage': robot_status, 'assigned_to': robot_name if robot_status != 'idle' else None})
        self.save_robot_status()

    def _wait_for_stage_completion(self, robot_name: str, expected_task_id: str, timeout: float) -> bool:
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            is_completed = False
            with self.lock:
                robot_state = self.robot_status.get(robot_name)
                is_completed = robot_state and robot_state.get('task_id') == expected_task_id and robot_state.get('status') == 'stage_completed'
            if is_completed: return True
            rospy.sleep(0.1)
        rospy.logerr(f"Timeout! Robot '{robot_name}' không báo cáo hoàn thành cho task '{expected_task_id}'.")
        return False

    def send_move_command(self, robot_name: str, target_location: Dict) -> bool:
        try:
            req = self.RequestPathRequest(robot_name=str(robot_name), end_point=Point(x=float(target_location['x']), y=float(target_location['y']), z=0.0))
            response = self.path_provider_client(req)
            if response.success and response.path.poses:
                self.path_publishers[robot_name].publish(response.path)
                return True
            rospy.logerr(f"[Path FAILED] Service cho '{robot_name}' trả về thất bại: '{response.message}'")
            return False
        except Exception as e:
            rospy.logerr(f"[Path FAILED] Lỗi khi gọi service cho '{robot_name}': {e}")
            return False

    def handle_task_failure(self, task_id: str, robot_name: str, reason: str = "UNKNOWN"):
        rospy.logerr(f"Lỗi nhiệm vụ '{task_id}' cho '{robot_name}'. Lý do: {reason}. Đặt lại trạng thái.")
        with self.lock: # Lock để đảm bảo tất cả các cập nhật trạng thái là nguyên tử
            if robot_name in self.robot_status:
                self.robot_status[robot_name]['status'] = 'idle'
                self.robot_status[robot_name]['task_id'] = None
            
            all_tasks = self._read_json_file(self.task_file)
            task_found = False
            for task in all_tasks:
                if task.get('id') == task_id:
                    task_found = True
                    retry_count = task.get('retries', 0) + 1
                    if retry_count >= MAX_RETRIES:
                        task['status'] = 'FAILED'
                        task['current_stage'] = f'{reason}_PERMANENTLY'
                    else:
                        task['status'] = 'PENDING'
                        task['assigned_to'] = None
                    task['retries'] = retry_count
                    break
            
            if task_found:
                self._write_json_file(self.task_file, all_tasks)

        self.save_robot_status()

    def run(self):
        rate = rospy.Rate(1)
        rospy.loginfo("TaskManager đã sẵn sàng (chế độ Worker-per-Robot).")
        while not rospy.is_shutdown():
            try:
                # Thực hiện các thao tác đọc và cập nhật trong lock để đảm bảo nhất quán
                with self.lock:
                    all_tasks = self._read_json_file(self.task_file)
                    pending_tasks = [t for t in all_tasks if t.get('status') == 'PENDING']
                    idle_robots = {name: data for name, data in self.robot_status.items() if data.get('status') == 'idle'}
                
                if not pending_tasks or not idle_robots:
                    rate.sleep()
                    continue

                pending_tasks.sort(key=lambda t: t.get('last_updated', ''))
                
                available_robots = list(idle_robots.keys())
                for task in pending_tasks:
                    if not available_robots: break

                    start_loc = task.get('pickup_location') or task.get('target_location')
                    if not start_loc: continue
                    
                    start_point = Point(x=start_loc['x'], y=start_loc['y'], z=0)
                    best_robot, min_distance = None, float('inf')

                    for robot_name in available_robots:
                        with self.lock:
                            # Kiểm tra xem có luồng nào đang chạy cho robot này không
                            if self.worker_threads.get(robot_name) and self.worker_threads[robot_name].is_alive():
                                continue
                        
                        robot_data = idle_robots[robot_name]
                        if not robot_data.get('current_location'): continue
                        
                        dist = math.hypot(robot_data['current_location'].x - start_point.x, robot_data['current_location'].y - start_point.y)
                        if dist < min_distance:
                            min_distance, best_robot = dist, robot_name
                    
                    if best_robot:
                        rospy.loginfo(f"Gán task '{task['id']}' cho '{best_robot}'. Khởi tạo luồng phụ.")
                        
                        # Cập nhật trạng thái và khởi động luồng
                        with self.lock:
                            self.robot_status[best_robot]['status'] = 'busy'
                            self.robot_status[best_robot]['task_id'] = task['id']
                            
                            thread = threading.Thread(target=self._execute_task_thread, args=(task, best_robot))
                            thread.daemon = True
                            self.worker_threads[best_robot] = thread

                        # Lưu trạng thái và cập nhật file task SAU KHI đã thoát lock
                        self.save_robot_status()
                        self.update_task_in_file(task['id'], {'status': 'ASSIGNED'})
                        
                        thread.start()
                        available_robots.remove(best_robot)

                rate.sleep()
            except Exception as e:
                rospy.logerr(f"Lỗi nghiêm trọng trong vòng lặp chính: {e}")
                traceback.print_exc()
                rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('task_manager_node', anonymous=True)
        package_name = 'mir_control' # <<< THAY TÊN PACKAGE ROS CỦA BẠN VÀO ĐÂY
        try:
            r = rospkg.RosPack()
            package_path = r.get_path(package_name)
            DATA_DIR = os.path.join(package_path, 'data')
            os.makedirs(DATA_DIR, exist_ok=True)
        except rospkg.ResourceNotFound:
            rospy.logfatal(f"Không tìm thấy package '{package_name}'.")
            rospy.signal_shutdown("Lỗi không tìm thấy package.")
            exit()
        
        for filename in ['tasks.json', 'missions.json']:
            filepath = os.path.join(DATA_DIR, filename)
            if not os.path.exists(filepath):
                with open(filepath, 'w') as f: json.dump([], f)
        
        robot_status_filepath = os.path.join(DATA_DIR, 'robot_status.json')
        if not os.path.exists(robot_status_filepath):
            with open(robot_status_filepath, 'w') as f: json.dump({}, f)

        ROBOT_NAMES = ['robot1', 'robot2', 'robot3']
        manager = TaskManager(ROBOT_NAMES, DATA_DIR)
        manager.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TaskManager node đã bị tắt.")
    except Exception as e:
        rospy.logerr(f"TaskManager gặp lỗi nghiêm trọng: {e}")
        traceback.print_exc()