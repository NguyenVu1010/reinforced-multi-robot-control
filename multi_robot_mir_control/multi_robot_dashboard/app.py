#!/usr/bin/env python3
# app.py

import threading
import atexit
import os
import dash
from dash import dcc, html, Input, Output, State, no_update
import dash_bootstrap_components as dbc
from flask import jsonify

# --- Import các thành phần và trang ---
# Quan trọng: Import các biến `layout`, không phải các hàm `create_layout`
from components.sidebar import create_sidebar
from pages.page_start.layout import layout as start_page_layout
from pages.page_control.layout import layout as page_control_layout
from pages.page_map_editor.layout import layout as page_map_editor_layout

# Import các hàm đăng ký callback
from pages.page_start import callbacks as start_page_callbacks
from pages.page_control import callbacks as page_control_callbacks
from pages.page_map_editor import callbacks as page_map_editor_callbacks

# Import logic ROS
from ros_comms.ros_handler import RosHandler, DATA_LOCK, GLOBAL_DATA

# --- CẤU HÌNH CHUNG ---
# Sử dụng đường dẫn tuyệt đối để đảm bảo script chạy đúng từ mọi nơi
APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_LAUNCH_COMMAND = "roslaunch mir_gazebo multi_robot_simulation.launch"
ROBOT_NAMES = ['robot1', 'robot2', 'robot3']
# Giả sử thư mục `data` và `scripts` (chứa app.py) nằm trong cùng một thư mục cha
PROJECT_ROOT = os.path.dirname(APP_DIR)
MAP_DATA_PATH = os.path.join(PROJECT_ROOT, 'data', 'map_data.json')
PATH_LIBRARY_PATH = os.path.join(PROJECT_ROOT, 'data', 'path_library.json')
IMAGE_ASSET_PATH = os.path.join(APP_DIR, 'assets', 'map_output.png')

# =============================================================================
# KHỞI TẠO ỨNG DỤNG DASH
# =============================================================================
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.BOOTSTRAP, dbc.icons.FONT_AWESOME], suppress_callback_exceptions=True)
server = app.server
app.title = "Multi-Robot Dashboard"

# Khởi tạo đối tượng RosHandler
ros_handler = RosHandler(ROS_LAUNCH_COMMAND, ROBOT_NAMES)

# =============================================================================
# ĐỊNH NGHĨA LAYOUT TĨNH (STATIC LAYOUT) - GIẢI QUYẾT LỖI KEYERROR VÀ RESET
# =============================================================================
app.layout = html.Div([
    # Các component vô hình để quản lý trạng thái và URL
    dcc.Location(id="url", refresh=False),
    dcc.Store(id='app-status-store', storage_type='memory'),
    dcc.Interval(id="app-status-interval", interval=1000, n_intervals=0),

    # Container cho trang Start (ban đầu hiển thị)
    html.Div(
        id='start-page-container',
        children=start_page_layout,
        style={'display': 'block'}
    ),

    # Container cho Dashboard chính (ban đầu bị ẩn)
    html.Div(
        id='dashboard-container',
        children=[
            html.Div(id="sidebar-wrapper"),
            # Vùng content này sẽ chứa các trang con
            html.Div(id="page-content-wrapper", className="content-base", children=[
                # Đặt layout của tất cả các trang con vào đây và ẩn chúng đi
                html.Div(id='page-control-wrapper', children=page_control_layout, style={'display': 'none'}),
                html.Div(id='page-map-editor-wrapper', children=page_map_editor_layout, style={'display': 'none'}),
            ])
        ],
        style={'display': 'none'}
    )
])

# =============================================================================
# CALLBACKS CHÍNH CỦA ỨNG DỤNG
# =============================================================================

# Callback 1: Cập nhật dcc.Store từ GLOBAL_DATA
@app.callback(
    Output('app-status-store', 'data'),
    Input('app-status-interval', 'n_intervals')
)
def update_app_status_store(n):
    with DATA_LOCK:
        status_data = {
            'connected': GLOBAL_DATA.get('ros_connected', False),
            'launched': GLOBAL_DATA.get('ros_launched', False),
            'message': GLOBAL_DATA.get('status_message', 'Initializing...')
        }
    return status_data

# Callback 2: Ẩn/hiện Start Page vs Dashboard
@app.callback(
    Output('start-page-container', 'style'),
    Output('dashboard-container', 'style'),
    Input('app-status-store', 'data')
)
def toggle_main_pages(status_data):
    if (status_data or {}).get('connected', False):
        return {'display': 'none'}, {'display': 'block'}
    return {'display': 'block'}, {'display': 'none'}

# Callback 3: Ẩn/hiện sidebar và đặt class cho content
@app.callback(
    Output('sidebar-wrapper', 'children'),
    Output('page-content-wrapper', 'className'),
    Input('app-status-store', 'data')
)
def toggle_sidebar_and_style(status_data):
    if (status_data or {}).get('connected', False):
        return create_sidebar(), 'content' # Thêm class 'content'
    else:
        return None, '' # Không có class

# Callback 4: Router - Chỉ ẩn/hiện các trang con
@app.callback(
    Output('page-control-wrapper', 'style'),
    Output('page-map-editor-wrapper', 'style'),
    Input('url', 'pathname')
)
def router(pathname):
    # Router này chỉ thay đổi style, không render lại layout
    if pathname == "/map-editor":
        return {'display': 'none'}, {'display': 'block'}
    # Mặc định (bao gồm cả "/") là trang control
    return {'display': 'block'}, {'display': 'none'}

# Callback 5: Tự động chuyển hướng URL
@app.callback(
    Output("url", "pathname"),
    Input("app-status-store", "data"),
    State("url", "pathname"), # <<< Thêm State để biết trang hiện tại
    prevent_initial_call=True
)
def redirect_on_connect(status_data, current_pathname):
    """
    Callback này chỉ có một nhiệm vụ: chuyển hướng người dùng từ trang Start
    đến trang chính MỘT LẦN DUY NHẤT sau khi kết nối thành công.
    """
    ctx = dash.callback_context
    # Đảm bảo callback chỉ chạy khi store thực sự thay đổi
    if not ctx.triggered or ctx.triggered[0]['prop_id'] != 'app-status-store.data':
        return no_update

    status_data = status_data or {}
    is_connected = status_data.get('connected', False)

    # --- LOGIC MỚI ---
    # Chỉ chuyển hướng nếu:
    # 1. Vừa kết nối thành công (is_connected = True)
    # 2. VÀ người dùng vẫn đang ở trang Start (current_pathname != "/" và != "/map-editor")
    #    (Chúng ta giả định trang Start không có URL cụ thể hoặc là một URL khác)
    if is_connected and (current_pathname != "/" and current_pathname != "/map-editor"):
        print(f"DEBUG: Connection successful. Redirecting from '{current_pathname}' to '/'...")
        return "/" # Chuyển hướng đến trang chính
    
    # Trong mọi trường hợp khác, không làm gì cả để người dùng tự do điều hướng
    return no_update

# --- ĐĂNG KÝ CÁC CALLBACK TỪ CÁC FILE RIÊNG BIỆT ---
print("INFO: Registering callbacks from external files...")
start_page_callbacks.register_callbacks(app, ros_handler)
page_control_callbacks.register_callbacks(app, MAP_DATA_PATH, PATH_LIBRARY_PATH, IMAGE_ASSET_PATH, pixels_per_meter=20.0)
page_map_editor_callbacks.register_callbacks(app)
print("SUCCESS: All external callbacks registered.")
@server.route('/dynamic-data')
def get_dynamic_data():
    """Cung cấp tọa độ robot/lidar mới nhất dưới dạng JSON."""
    try:
        with DATA_LOCK:
            poses = GLOBAL_DATA.get('poses', {})
            lidar_data = GLOBAL_DATA.get('lidar', {})
        
        return jsonify({'poses': poses, 'lidar': lidar_data})
    except Exception as e:
        print(f"Error in /dynamic-data endpoint: {e}")
        return jsonify({"error": str(e)}), 500
# =============================================================================
# HÀM CHÍNH VÀ QUẢN LÝ VÒNG ĐỜI
# =============================================================================
def main():
    ros_handler_thread = threading.Thread(target=ros_handler.main_loop)
    ros_handler_thread.daemon = True
    ros_handler_thread.start()
    atexit.register(ros_handler.shutdown)
    
    print("Khởi động server Dash trên http://0.0.0.0:8050")
    app.run_server(debug=False, host='0.0.0.0', port=8050)

if __name__ == '__main__':
    main()