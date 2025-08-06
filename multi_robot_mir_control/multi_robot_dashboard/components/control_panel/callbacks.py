# multi_robot_dashboard/components/control_panel/callbacks.py

from dash import Dash, Input, Output, State, html, dcc, callback_context, ALL, no_update
import dash_bootstrap_components as dbc
import json
import uuid
from datetime import datetime
import os
from typing import List, Dict, Tuple

# --- CẤU HÌNH ĐƯỜNG DẪN FILE ---
# Sử dụng phương pháp an toàn để xác định đường dẫn, bất kể file app.py chạy từ đâu
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.join(BASE_DIR, '..', '..', '..')
MISSION_TEMPLATE_FILE = os.path.join(PACKAGE_ROOT, 'data', 'missions.json')
TASK_QUEUE_FILE = os.path.join(PACKAGE_ROOT, 'data', 'tasks.json')


# --- HÀM TIỆN ÍCH ---
def read_json_file(filepath: str) -> List[Dict]:
    """Đọc file JSON an toàn, trả về list rỗng nếu có lỗi."""
    if not os.path.exists(filepath):
        return []
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = json.load(f)
            return data if isinstance(data, list) else []
    except (FileNotFoundError, json.JSONDecodeError):
        return []

def write_json_file(filepath: str, data: List[Dict]) -> Tuple[bool, str]:
    """Ghi dữ liệu vào file JSON, trả về (thành công, thông báo lỗi)."""
    try:
        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return True, ""
    except Exception as e:
        return False, str(e)

def create_task_from_template(template: Dict) -> Dict:
    """Tạo một task mới có thể thực thi từ một template."""
    new_task = {k: v for k, v in template.items() if k != 'template_id'}
    new_task.update({
        "id": f"task_{uuid.uuid4().hex[:6]}",
        "status": "PENDING",
        "assigned_to": None,
        "current_stage": None,
        "last_updated": datetime.utcnow().isoformat() + "Z"
    })
    return new_task

# --- ĐĂNG KÝ CALLBACKS VỚI ỨNG DỤNG ---
def register_callbacks(app: Dash):

    # ===============================================================
    # CALLBACKS CHO MODAL TẠO TEMPLATE
    # ===============================================================

    @app.callback(
        Output("cp-create-mission-modal", "is_open"),
        Input("cp-open-create-modal-btn", "n_clicks"),
        Input("cp-close-modal-button", "n_clicks"),
        [State("cp-create-mission-modal", "is_open")],
        prevent_initial_call=True,
    )
    def toggle_modal(n_open, n_close, is_open):
        if callback_context.triggered_id in ["cp-open-create-modal-btn", "cp-close-modal-button"]:
            return not is_open
        return is_open

    @app.callback(
        Output('cp-task-form-inputs', 'children'),
        Input('cp-task-type-dropdown', 'value')
    )
    def render_task_form_inputs(task_type: str) -> html.Div:
        if task_type == 'GOTO':
            return html.Div([
                dbc.Row([dbc.Label("Template Name:", width=4), dbc.Col(dbc.Input(id='cp-goto-name-input', type='text', placeholder='e.g., Go to Charging Station'), width=8)], className="mb-2"),
                dbc.Row([dbc.Label("Target X:", width=4), dbc.Col(dbc.Input(id='cp-goto-x-input', type='number'), width=8)], className="mb-2"),
                dbc.Row([dbc.Label("Target Y:", width=4), dbc.Col(dbc.Input(id='cp-goto-y-input', type='number'), width=8)])
            ])
        elif task_type == 'DELIVERY':
            return html.Div([
                dbc.Row([dbc.Label("Template Name:", width=4), dbc.Col(dbc.Input(id='cp-delivery-name-input', type='text', placeholder='e.g., Deliver Parts A->B'), width=8)], className="mb-2"),
                html.Hr(),
                html.H6("Pickup", className="mt-3"),
                dbc.Row([dbc.Label("X:", width=4), dbc.Col(dbc.Input(id='cp-pickup-x-input', type='number'), width=8)], className="mb-2"),
                dbc.Row([dbc.Label("Y:", width=4), dbc.Col(dbc.Input(id='cp-pickup-y-input', type='number'), width=8)], className="mb-2"),
                html.Hr(),
                html.H6("Dropoff"),
                dbc.Row([dbc.Label("X:", width=4), dbc.Col(dbc.Input(id='cp-dropoff-x-input', type='number'), width=8)], className="mb-2"),
                dbc.Row([dbc.Label("Y:", width=4), dbc.Col(dbc.Input(id='cp-dropoff-y-input', type='number'), width=8)])
            ])
        return html.Div()

    @app.callback(
        Output('cp-general-alert', 'children', allow_duplicate=True),
        Output('cp-data-trigger-store', 'data', allow_duplicate=True),
        Output("cp-create-mission-modal", "is_open", allow_duplicate=True),
        Input('cp-submit-mission-button', 'n_clicks'),
        [
            State('cp-task-type-dropdown', 'value'),
            # THAY ĐỔI LỚN: Chỉ lấy State từ vùng chứa form
            State('cp-task-form-inputs', 'children')
        ],
        prevent_initial_call=True
    )
    def handle_create_mission_template(n_clicks: int, task_type: str, form_children: Dict):
        """
        Callback này xử lý việc tạo và lưu một mẫu nhiệm vụ mới.
        Nó đọc dữ liệu từ cấu trúc 'children' của form thay vì các State riêng lẻ.
        """
        
        if not n_clicks:
            return no_update, no_update, no_update

        trigger_data = datetime.utcnow().isoformat()
        new_template = {"template_id": f"tmpl_{uuid.uuid4().hex[:6]}"}
        
        try:
            # Trích xuất dữ liệu từ cấu trúc children
            # form_children là một dict hoặc list of dicts mô tả layout
            # Chúng ta cần duyệt qua nó để tìm các giá trị
            
            props = {}
            # form_children['props']['children'] là một list các dbc.Row
            for row in form_children['props']['children']:
                # Bỏ qua các thẻ Hr
                if row['type'] == 'Hr':
                    continue
                # Bỏ qua các thẻ H6
                if row['type'] == 'H6':
                    continue
                    
                # Mỗi row là một dict, children[1] là dbc.Col, children[0] là dbc.Input
                input_component = row['props']['children'][1]['props']['children']
                component_id = input_component['props']['id']
                value = input_component['props'].get('value')
                props[component_id] = value

            # Xây dựng và xác thực template
            is_valid = False
            if task_type == 'GOTO':
                goto_name = props.get('cp-goto-name-input')
                goto_x = props.get('cp-goto-x-input')
                goto_y = props.get('cp-goto-y-input')
                if goto_name and goto_x is not None and goto_y is not None:
                    new_template.update({"type": "GOTO", "name": goto_name, "target_location": {"x": float(goto_x), "y": float(goto_y)}})
                    is_valid = True
            
            elif task_type == 'DELIVERY':
                delivery_name = props.get('cp-delivery-name-input')
                pickup_x = props.get('cp-pickup-x-input')
                pickup_y = props.get('cp-pickup-y-input')
                dropoff_x = props.get('cp-dropoff-x-input')
                dropoff_y = props.get('cp-dropoff-y-input')
                if delivery_name and pickup_x is not None and pickup_y is not None and dropoff_x is not None and dropoff_y is not None:
                    new_template.update({"type": "DELIVERY", "name": delivery_name, "pickup_location": {"x": float(pickup_x), "y": float(pickup_y)}, "dropoff_location": {"x": float(dropoff_x), "y": float(dropoff_y)}})
                    is_valid = True
            
            if not is_valid:
                alert = dbc.Alert("Please fill all required fields.", color="danger", duration=4000)
                return alert, no_update, no_update

            # Ghi file
            mission_templates = read_json_file(MISSION_TEMPLATE_FILE)
            mission_templates.append(new_template)
            success, error_msg = write_json_file(MISSION_TEMPLATE_FILE, mission_templates)
            
            if success:
                alert = dbc.Alert(f"Template '{new_template['name']}' created!", color="success", duration=3000)
                return alert, trigger_data, False
            else:
                alert = dbc.Alert(f"Error saving template: {error_msg}", color="danger")
                return alert, no_update, True

        except (KeyError, IndexError) as e:
            # Bắt lỗi nếu cấu trúc children không như mong đợi
            alert = dbc.Alert(f"Error parsing form data: {e}. Please check the layout.", color="danger")
            return alert, no_update, no_update

    # ===============================================================
    # CALLBACK CẬP NHẬT TOÀN BỘ GIAO DIỆN
    # ===============================================================

    @app.callback(
        Output('mission-templates-tbody', 'children'),
        Output('task-queue-tbody', 'children'),
        Input('cp-interval-component', 'n_intervals'),
        Input('cp-data-trigger-store', 'data')
    )
    def update_all_tables(n, trigger_data):
        # --- Bảng mẫu nhiệm vụ ---
        mission_templates = read_json_file(MISSION_TEMPLATE_FILE)
        template_rows = [html.Tr(html.Td("No templates found.", colSpan=3, className="text-center text-muted"))]
        if mission_templates:
            template_rows = []
            for template in mission_templates:
                template_rows.append(html.Tr([
                    html.Td(template.get('name', 'Unnamed')),
                    html.Td(html.Span(template.get('type'), className=f"badge bg-info text-dark")),
                    html.Td([
                        dbc.Button(html.I(className="fas fa-play"), id={'type': 'run-template-btn', 'index': template['template_id']}, title="Run this template", color="success", size="sm", className="me-1"),
                        dbc.Button(html.I(className="fas fa-trash-alt"), id={'type': 'delete-template-btn', 'index': template['template_id']}, title="Delete Template", color="danger", size="sm"),
                    ], className="text-center")
                ]))
        
        # --- Bảng hàng đợi thực thi ---
        task_queue = read_json_file(TASK_QUEUE_FILE)
        queue_rows = [html.Tr(html.Td("Execution queue is empty.", colSpan=4, className="text-center text-muted"))]
        tasks_to_display = [t for t in task_queue if t.get('status') != 'COMPLETED']
        if tasks_to_display:
            queue_rows = []
            for task in sorted(tasks_to_display, key=lambda t: t['last_updated'], reverse=True):
                robot_info = f"({task.get('assigned_to', 'unassigned')})" if task.get('assigned_to') else ""
                status_colors = {"PENDING": "primary", "ACTIVE": "warning", "FAILED": "danger"}
                status_color = status_colors.get(task.get('status'), "secondary")
                
                # SỬA LỖI 2: Logic render nút hành động linh hoạt
                action_button = None
                task_status = task.get('status')
                if task_status in ['PENDING', 'ACTIVE']:
                    action_button = dbc.Button(html.I(className="fas fa-times"), id={'type': 'cancel-task-btn', 'index': task['id']}, title="Cancel Task (set to FAILED)", color="warning", size="sm")
                elif task_status == 'FAILED':
                    action_button = dbc.Button(html.I(className="fas fa-trash-alt"), id={'type': 'clear-task-btn', 'index': task['id']}, title="Clear this failed task", color="danger", size="sm")
                
                queue_rows.append(html.Tr([
                    html.Td(task['id']),
                    html.Td(f"{task.get('name')} {robot_info}"),
                    html.Td(html.Span(task.get('status'), className=f"badge bg-{status_color}")),
                    html.Td(action_button, className="text-center")
                ]))
        return template_rows, queue_rows


    # ===============================================================
    # CALLBACKS XỬ LÝ CÁC HÀNH ĐỘNG KHÁC
    # ===============================================================

    @app.callback(
        Output('cp-general-alert', 'children', allow_duplicate=True),
        Output('cp-data-trigger-store', 'data', allow_duplicate=True),
        Input('cp-stop-all-btn', 'n_clicks'),
        prevent_initial_call=True
    )
    def stop_all_tasks(n_clicks):
        tasks = read_json_file(TASK_QUEUE_FILE)
        tasks_stopped = 0
        for task in tasks:
            if task.get('status') in ['PENDING', 'ACTIVE']:
                task['status'] = 'FAILED'
                task['current_stage'] = 'STOPPED_BY_USER'
                task['last_updated'] = datetime.utcnow().isoformat() + "Z"
                tasks_stopped += 1
        if tasks_stopped > 0:
            success, error_msg = write_json_file(TASK_QUEUE_FILE, tasks)
            alert = dbc.Alert(f"Stopped {tasks_stopped} active/pending tasks.", color="warning", duration=4000) if success else dbc.Alert(f"Error: {error_msg}", color="danger")
            return alert, datetime.utcnow().isoformat()
        return dbc.Alert("No active tasks to stop.", color="info", duration=3000), no_update

    @app.callback(
        Output('cp-data-trigger-store', 'data', allow_duplicate=True),
        Input({'type': 'run-template-btn', 'index': ALL}, 'n_clicks'),
        Input({'type': 'delete-template-btn', 'index': ALL}, 'n_clicks'),
        Input({'type': 'cancel-task-btn', 'index': ALL}, 'n_clicks'),
        Input({'type': 'clear-task-btn', 'index': ALL}, 'n_clicks'),
        prevent_initial_call=True
    )
    def handle_dynamic_buttons(run_clicks, delete_template_clicks, cancel_clicks, clear_task_clicks):
        ctx = callback_context
        if not ctx.triggered or not ctx.triggered[0]['value']:
            return no_update
        
        button_id_str = ctx.triggered[0]['prop_id'].split('.')[0]
        button_info = json.loads(button_id_str)
        action_type = button_info['type']
        item_id = button_info['index']

        if action_type == 'run-template-btn':
            mission_templates = read_json_file(MISSION_TEMPLATE_FILE)
            template_to_run = next((t for t in mission_templates if t['template_id'] == item_id), None)
            if template_to_run:
                new_task = create_task_from_template(template_to_run)
                task_queue = read_json_file(TASK_QUEUE_FILE)
                task_queue.append(new_task)
                write_json_file(TASK_QUEUE_FILE, task_queue)

        elif action_type == 'delete-template-btn':
            mission_templates = read_json_file(MISSION_TEMPLATE_FILE)
            templates_after_delete = [t for t in mission_templates if t['template_id'] != item_id]
            write_json_file(MISSION_TEMPLATE_FILE, templates_after_delete)

        elif action_type == 'cancel-task-btn':
            task_queue = read_json_file(TASK_QUEUE_FILE)
            for task in task_queue:
                if task['id'] == item_id and task.get('status') in ['PENDING', 'ACTIVE']:
                    task['status'] = 'FAILED'
                    task['current_stage'] = 'CANCELED_BY_USER'
                    task['last_updated'] = datetime.utcnow().isoformat() + "Z"
                    break
            write_json_file(TASK_QUEUE_FILE, task_queue)
        
        elif action_type == 'clear-task-btn':
            task_queue = read_json_file(TASK_QUEUE_FILE)
            tasks_after_clear = [t for t in task_queue if t.get('id') != item_id]
            write_json_file(TASK_QUEUE_FILE, tasks_after_clear)
            
        return datetime.utcnow().isoformat()