# layout.py

from dash import dcc, html
import dash_bootstrap_components as dbc
import os
import json
import base64
from PIL import Image
# Thêm các import cần thiết
import math
from scipy.spatial.distance import euclidean

# --- ĐỊNH NGHĨA ĐƯỜNG DẪN CỐ ĐỊNH ---
DEFAULT_IMAGE_PATH = 'src/mir_robot/mir_control/Map.png'
DEFAULT_JSON_PATH = 'src/mir_robot/mir_control/data/map_data.json'

# --- HÀM TÍNH TOÁN (chuyển từ callbacks.py sang đây) ---
def _calculate_circle_from_three_points(p1, p2, p3):
    x1, y1 = p1; x2, y2 = p2; x3, y3 = p3
    D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
    if abs(D) < 1e-8: return None
    Ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / D
    Uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / D
    center = (Ux, Uy); radius = euclidean(center, p1)
    return center, radius

def _get_arc_params(p1, mid_point, p2):
    calc = _calculate_circle_from_three_points(p1, mid_point, p2)
    if not calc: return None
    center, radius = calc
    start_angle_rad = math.atan2(-(p1[1] - center[1]), p1[0] - center[0])
    mid_angle_rad = math.atan2(-(mid_point[1] - center[1]), mid_point[0] - center[0])
    end_angle_rad = math.atan2(-(p2[1] - center[1]), p2[0] - center[0])
    norm_end = end_angle_rad - start_angle_rad
    norm_mid = mid_angle_rad - start_angle_rad
    if norm_end < 0: norm_end += 2 * math.pi
    if norm_mid < 0: norm_mid += 2 * math.pi
    sweep_angle_rad = norm_end if norm_mid < norm_end else (norm_end - (2 * math.pi))
    return {"center": center, "radius": radius, "start_angle_rad": start_angle_rad, "sweep_angle_rad": sweep_angle_rad, "mid_point": mid_point, "length": abs(sweep_angle_rad * radius)}

# --- HÀM HELPER ĐỂ TẢI DỮ LIỆU MẶC ĐỊNH ---
def load_default_image_data(path):
    if not os.path.exists(path):
        print(f"Cảnh báo: Không tìm thấy ảnh mặc định tại '{path}'. Sử dụng nền trắng.")
        return {'contents': None, 'width': 800, 'height': 600}
    try:
        pil_image = Image.open(path)
        width, height = pil_image.size
        with open(path, "rb") as f:
            encoded_string = base64.b64encode(f.read()).decode()
        file_ext = os.path.splitext(path)[1][1:].lower()
        if file_ext == 'jpg': file_ext = 'jpeg'
        contents = f"data:image/{file_ext};base64,{encoded_string}"
        return {'contents': contents, 'width': width, 'height': height}
    except Exception as e:
        print(f"Lỗi tải ảnh mặc định '{path}': {e}")
        return {'contents': None, 'width': 800, 'height': 600}

# CẬP NHẬT HÀM NÀY
def load_default_graph_data(path):
    if not os.path.exists(path):
        print(f"Cảnh báo: Không tìm thấy file JSON mặc định tại '{path}'. Bắt đầu với đồ thị trống.")
        return {'nodes': {}, 'edges': []}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        nodes = {n: tuple(c) for n, c in data.get('nodes', {}).items()}
        edges_raw = data.get('edges', [])
        
        recalculated_edges = []
        for edge in edges_raw:
            new_edge = edge.copy()
            new_edge.setdefault('type', 'line')
            new_edge.setdefault('direction', 'thuan')
            if new_edge['type'] == 'arc':
                p1 = nodes.get(new_edge.get('from'))
                p2 = nodes.get(new_edge.get('to'))
                mid = new_edge.get('mid_point')
                if p1 and p2 and mid:
                    params = _get_arc_params(p1, tuple(mid), p2) # Đảm bảo mid_point là tuple
                    if params:
                        new_edge.update(params)
                    else: # Chuyển về đường thẳng nếu 3 điểm thẳng hàng
                        print(f"Cảnh báo: Cung tròn từ {new_edge['from']} đến {new_edge['to']} không hợp lệ. Chuyển thành đường thẳng.")
                        new_edge['type'] = 'line'
                        for key in ['center', 'radius', 'mid_point', 'start_angle_rad', 'sweep_angle_rad']: new_edge.pop(key, None)
                        new_edge['length'] = euclidean(p1, p2)
            recalculated_edges.append(new_edge)
        
        return {'nodes': nodes, 'edges': recalculated_edges}
    except Exception as e:
        print(f"Lỗi tải hoặc xử lý file JSON mặc định '{path}': {e}")
        return {'nodes': {}, 'edges': []}

# --- KHỞI TẠO DỮ LIỆU BAN ĐẦU ---
initial_image_data = load_default_image_data(DEFAULT_IMAGE_PATH)
initial_graph_data = load_default_graph_data(DEFAULT_JSON_PATH)

# --- BỐ CỤC (LAYOUT) ---
# Bố cục giữ nguyên, không cần thay đổi
layout = dbc.Container([
    dcc.Store(id='graph-data-store', data=initial_graph_data),
    dcc.Store(id='history-store', data=[]),
    dcc.Store(id='ui-state-store', data={'mode': 'normal', 'temp_node_pos': None, 'edge_start_node': None, 'arc_mid_pos': None}),
    dcc.Store(id='image-store', data=initial_image_data),
    dcc.Store(id='edit-edge-info-store', data=None),

    dcc.Download(id="download-json"),
    dcc.Download(id="download-image"),

    html.H1("Trình Soạn Thảo Đồ Thị Giao Thông", className="text-center my-3"),

    dbc.Row([
        dbc.Col(
            dcc.Graph(id='map-canvas', config={'staticPlot': False, 'scrollZoom': True}),
            width=8, style={'border': '1px solid #ccc', 'height': '80vh', 'overflow': 'auto'}
        ),
        dbc.Col([
            dbc.Card([dbc.CardHeader("Hành động"), dbc.CardBody([
                dbc.Button("Hoàn tác (Undo)", id="undo-button", color="secondary", className="w-100 mb-2"),
                dbc.Button("Bật Chế độ Xóa", id="delete-mode-button", color="secondary", className="w-100")
            ])]),

            dbc.Card([dbc.CardHeader("File Operations"), dbc.CardBody([
                html.P(f"Ảnh nền: {os.path.basename(DEFAULT_IMAGE_PATH)}", className="small text-muted"),
                html.P(f"File dữ liệu: {os.path.basename(DEFAULT_JSON_PATH)}", className="small text-muted"),
                html.Hr(),
                dbc.Input(id="json-filename-input", placeholder="Nhập tên file JSON để lưu...", value="map_data_output.json", className="mb-2"),
                dbc.Button("Lưu Dữ liệu (JSON)", id="save-json-button", className="w-100 mb-2"),
                dbc.Input(id="output-filename-input", placeholder="Nhập tên file ảnh để lưu...", value="map_output.png", className="mb-2"),
                dbc.Button("Lưu Ảnh Map", id="save-image-button", className="w-100"),
            ])], className="my-3"),

            dbc.Card([dbc.CardHeader("Thêm/Sửa Đối Tượng"), dbc.CardBody([
                html.P("Tên node mới:"),
                dbc.Input(id="node-name-input", placeholder="Nhập tên node..."),
                dbc.Button("Thêm node tại điểm đã chọn", id="add-node-button", color="primary", className="w-100 mt-2 mb-3"),
                html.Hr(),
                html.P("Loại Cạnh/Cung:"),
                dbc.RadioItems(
                    id='edge-type-radio',
                    options=[{'label': 'Đường thẳng', 'value': 'line'}, {'label': 'Cung tròn (3 điểm)', 'value': 'arc'}],
                    value='line',
                ),
                html.P("Hướng đi (làn):", className="mt-3"),
                 dbc.RadioItems(
                    id='direction-radio',
                    options=[{'label': 'thuận', 'value': 'thuan'}, {'label': 'nghịch', 'value': 'nghich'}],
                    value='thuan',
                ),
            ])]),
        ], width=4)
    ]),

    dbc.Alert(id="status-bar", children="Sẵn sàng. Đã tải dữ liệu mặc định.", color="info", className="mt-2"),

    dbc.Modal([
        dbc.ModalHeader("Sửa Hướng Cạnh"),
        dbc.ModalBody([
            html.P("Hướng đi mới:"),
            dbc.RadioItems(id='edit-direction-radio', options=[{'label': 'thuận', 'value': 'thuan'}, {'label': 'nghịch', 'value': 'nghich'}], value='thuan')
        ]),
        dbc.ModalFooter([
            dbc.Button("OK", id="edit-edge-ok-button", color="primary"),
            dbc.Button("Hủy", id="edit-edge-cancel-button", color="secondary")
        ])
    ], id="edit-edge-modal", is_open=False)
], fluid=True)