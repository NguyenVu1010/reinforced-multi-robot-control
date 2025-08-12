# components/control_panel/callbacks.py

from dash import Input, Output, html, State, ALL, ctx
import dash_bootstrap_components as dbc
import time

# --- MÔ PHỎNG DỮ LIỆU ---
# Trong thực tế, bạn sẽ lấy dữ liệu này từ ros_handler hoặc API
AVAILABLE_MISSIONS = [
    {"guid": "mission-guid-001", "name": "Alpha Reconnaissance"},
    {"guid": "mission-guid-002", "name": "Beta Cargo Run"},
]
MISSION_QUEUE = []

# Hàm này bạn đã có, dùng để tạo một hàng trong bảng
def create_mission_row(mission):
    mission_id = mission.get("guid", "")
    return html.Tr([
        html.Td(mission_id),
        html.Td(mission.get("name", "")),
        html.Td([
            html.Button(html.I(className="fas fa-check"), id={"type": "addmission-btn", "index": mission_id}, n_clicks=0, className="btn btn-success btn-sm"),
            html.Button(html.I(className="fas fa-eye"), id={"type": "viewmission-btn", "index": mission_id}, n_clicks=0, className="btn btn-primary btn-sm mx-1"),
            html.Button(html.I(className="fas fa-times"), id={"type": "stopmission-btn", "index": mission_id}, n_clicks=0, className="btn btn-danger btn-sm"),
        ]),
    ])

def register_callbacks(app):
    """Đăng ký tất cả callbacks cho Control Panel."""

    # Callback cập nhật danh sách nhiệm vụ và hàng đợi
    @app.callback(
        Output('mission-list-container', 'children'),
        Output('mission-queue-container', 'children'),
        # Giả sử có một interval chung để cập nhật
        Input('interval-update-ui', 'n_intervals') 
    )
    def update_mission_lists(n):
        # --- ĐÂY LÀ NƠI BẠN SẼ LẤY DỮ LIỆU THỰC TỪ GLOBAL_DATA ---
        
        # Mô phỏng
        mission_list_rows = [create_mission_row(m) for m in AVAILABLE_MISSIONS]
        
        queue_rows = []
        for item in MISSION_QUEUE:
            row = html.Tr([
                html.Td(item['queue_id']),
                html.Td(item['mission_name']),
                html.Td(html.Button("Cancel", className="btn btn-warning btn-sm"))
            ])
            queue_rows.append(row)

        return mission_list_rows, queue_rows

    # Callback xử lý khi nhấn nút "Add to Queue"
    @app.callback(
        Output('api-addmissions-response-message', 'children'),
        Input({'type': 'addmission-btn', 'index': ALL}, 'n_clicks'),
        prevent_initial_call=True
    )
    def add_mission_to_queue(n_clicks):
        # Xác định nút nào đã được nhấn
        triggered_id = ctx.triggered_id
        if not triggered_id or all(c == 0 for c in n_clicks):
            return ""

        mission_guid = triggered_id['index']
        
        # Tìm nhiệm vụ tương ứng
        mission_to_add = next((m for m in AVAILABLE_MISSIONS if m['guid'] == mission_guid), None)
        
        if mission_to_add:
            # Mô phỏng việc thêm vào hàng đợi
            new_queue_item = {
                "queue_id": f"Q-{int(time.time()) % 1000}",
                "mission_guid": mission_to_add['guid'],
                "mission_name": mission_to_add['name'],
            }
            MISSION_QUEUE.append(new_queue_item)
            
            return dbc.Alert(f"Đã thêm '{mission_to_add['name']}' vào hàng đợi!", color="success", duration=3000)
        
        return dbc.Alert("Lỗi: Không tìm thấy nhiệm vụ.", color="danger", duration=3000)
    
    # --- THÊM CÁC CALLBACK KHÁC CHO CÁC NÚT BẤM KHÁC Ở ĐÂY ---
    # Ví dụ: Mở modal khi nhấn "Create"
    # @app.callback(...)
    # def handle_create_button(...)