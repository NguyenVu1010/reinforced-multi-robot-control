# components/map_viewer/callbacks.py

from dash.dependencies import Input, Output
import plotly.graph_objects as go
from PIL import Image
import json
import pandas as pd
import numpy as np
import os
import traceback
try:
    from ros_comms.ros_handler import GLOBAL_DATA, DATA_LOCK
except ImportError:
    import threading
    print("WARN (MapViewer): Running in standalone/debug mode.")
    DATA_LOCK = threading.Lock()
    GLOBAL_DATA = {}

# --- CÁC HÀM TIỆN ÍCH (KHÔNG THAY ĐỔI) ---

def convert_pixel_to_metric(px, py, image_width_px, image_height_px, pixels_per_meter):
    """
    Chuyển đổi tọa độ từ hệ pixel (gốc trên-trái, y hướng xuống)
    sang hệ mét (gốc ở tâm ảnh, y hướng lên).
    Hàm này VẪN CẦN THIẾT để xử lý dữ liệu tĩnh từ file JSON.
    """
    if pixels_per_meter == 0:
        return 0, 0
    center_x_px = image_width_px / 2.0
    center_y_px = image_height_px / 2.0
    offset_x_px = px - center_x_px
    offset_y_px = py - center_y_px
    x_metric = offset_x_px / pixels_per_meter
    y_metric = -offset_y_px / pixels_per_meter
    return x_metric, y_metric

def load_node_data(map_data_path, image_width_px, image_height_px, pixels_per_meter):
    """
    Tải dữ liệu chỉ cho các node và chuyển đổi tọa độ.
    """
    try:
        with open(map_data_path, 'r', encoding='utf-8') as f: map_data = json.load(f)

        # Chỉ xử lý 'nodes'
        nodes_df = pd.DataFrame.from_dict(map_data['nodes'], orient='index', columns=['x_pixel', 'y_pixel'])
        
        # Chuyển đổi tọa độ node sang hệ mét
        metric_coords = nodes_df.apply(lambda row: convert_pixel_to_metric(row['x_pixel'], row['y_pixel'], image_width_px, image_height_px, pixels_per_meter), axis=1, result_type='expand')
        nodes_df[['x', 'y']] = metric_coords
            
        return nodes_df
    except Exception as e:
        print(f"ERROR: Failed to load node data from '{map_data_path}': {e}")
        traceback.print_exc()
        return pd.DataFrame()

def get_image_dimensions(image_path):
    try:
        with Image.open(image_path) as img: return img.size
    except FileNotFoundError:
        print(f"ERROR: Background image not found at '{image_path}'")
        return None, None

# --- HÀM ĐĂNG KÝ CALLBACK CHÍNH ---
def register_callbacks(app, map_data_file_path, path_library_path, image_asset_path, pixels_per_meter=20.0):
    
    # Phần khởi tạo ban đầu không đổi
    MAP_IMAGE_WIDTH_PX, MAP_IMAGE_HEIGHT_PX = get_image_dimensions(image_asset_path)
    if MAP_IMAGE_WIDTH_PX is None:
        print("CRITICAL (MapViewer): Cannot load image, coordinate conversion will fail.")
        MAP_IMAGE_WIDTH_PX, MAP_IMAGE_HEIGHT_PX = 1000, 1000 
    
    MAP_WIDTH_M = MAP_IMAGE_WIDTH_PX / pixels_per_meter
    MAP_HEIGHT_M = MAP_IMAGE_HEIGHT_PX / pixels_per_meter

    print("INFO (MapViewer): Loading and converting static map data...")
    NODES_DF = load_node_data(
        map_data_file_path,
        MAP_IMAGE_WIDTH_PX,
        MAP_IMAGE_HEIGHT_PX,
        pixels_per_meter
    )

    if NODES_DF.empty:
        print("CRITICAL (MapViewer): Static data failed to load.")

    @app.callback(
        Output('map-graph', 'figure'),
        Input('page-control-interval', 'n_intervals'),
        Input('layer-toggle-checklist', 'value')
    )
    def update_map_figure(n_intervals, selected_layers):
        
        if not isinstance(selected_layers, list):
            selected_layers = []

        fig = go.Figure()

        if NODES_DF.empty:
            return fig.update_layout(title_text="Error: Could not load map data.", plot_bgcolor='#EAEAEA')

        with DATA_LOCK:
            # Dữ liệu động này đã ở dạng mét
            poses = GLOBAL_DATA.get('poses', {})
            lidar_data = GLOBAL_DATA.get('lidar', {})
            robot_colors = GLOBAL_DATA.get('robot_colors', {})
            robot_list = list(poses.keys())

        if 'nodes' in selected_layers:
            node_hover_texts = [
                f"<b>Node: {idx}</b><br>Position: ({row.x:.2f} m, {row.y:.2f} m)<extra></extra>" 
                for idx, row in NODES_DF.iterrows()
            ]
            fig.add_trace(go.Scatter(
                x=NODES_DF['x'], y=NODES_DF['y'], mode='markers', 
                marker=dict(size=10, color='#1f77b4'), 
                hovertext=node_hover_texts, hoverinfo='text', name='Nodes'
            ))

        # *** CẬP NHẬT LOGIC VẼ DỮ LIỆU ĐỘNG ***
        if 'robots' in selected_layers:
            for name in robot_list:
                # Lidar (vẽ trước) - Dữ liệu đã ở dạng mét
                lidar_points_metric = lidar_data.get(name, [])
                if lidar_points_metric:
                    try:
                        # Sử dụng trực tiếp tọa độ mét, không cần chuyển đổi
                        lidar_x, lidar_y = zip(*lidar_points_metric)
                        fig.add_trace(go.Scatter(
                            x=list(lidar_x), y=list(lidar_y), mode='markers', 
                            marker=dict(size=3, color=robot_colors.get(name, 'black'), opacity=0.6), 
                            hoverinfo='none'
                        ))
                    except (ValueError, TypeError, IndexError):
                        continue
                
                # Robot (vẽ sau để nằm trên) - Dữ liệu đã ở dạng mét
                pose_metric = poses.get(name)
                if pose_metric and 'x' in pose_metric and 'y' in pose_metric:
                    # Sử dụng trực tiếp tọa độ mét, không cần chuyển đổi
                    x_m = pose_metric['x']
                    y_m = pose_metric['y']
                    hover_text = (f"<b>Robot: {name}</b><br>Position: ({x_m:.2f} m, {y_m:.2f} m)<br>Status: Idle<extra></extra>")
                    fig.add_trace(go.Scatter(
                        x=[x_m], y=[y_m], mode='markers', 
                        marker=dict(size=20, color=robot_colors.get(name, 'black'), symbol='diamond', line=dict(width=2, color='black')), 
                        hovertext=[hover_text], hoverinfo='text', name=name
                    ))
        
        # Cấu hình layout và ảnh nền (không thay đổi)
        padding_m = 5
        x_range = [NODES_DF['x'].min() - padding_m, NODES_DF['x'].max() + padding_m]
        y_range = [NODES_DF['y'].min() - padding_m, NODES_DF['y'].max() + padding_m]
        
        fig.update_layout(
            xaxis=dict(range=x_range, visible=False, scaleanchor="y", scaleratio=1),
            yaxis=dict(range=y_range, visible=False),
            showlegend=False, margin=dict(l=0, r=0, t=0, b=0), plot_bgcolor='white',
            hovermode='closest', hoverlabel=dict(bgcolor="white", font_size=14)
        )

        if 'map' in selected_layers and MAP_IMAGE_WIDTH_PX is not None:
            image_filename = os.path.basename(image_asset_path)
            fig.add_layout_image(
                dict(
                    source=app.get_asset_url(image_filename),
                    xref="x", yref="y",
                    x=-MAP_WIDTH_M / 2, y=MAP_HEIGHT_M / 2,
                    sizex=MAP_WIDTH_M, sizey=MAP_HEIGHT_M,
                    sizing="contain", opacity=0.5, layer="below"
                )
            )
            
        return fig
        