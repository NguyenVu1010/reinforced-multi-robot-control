from dash.dependencies import Input, Output, State
import plotly.graph_objects as go
from PIL import Image
import json
import pandas as pd
import numpy as np
import os
import traceback

# Giả định dữ liệu toàn cục (GLOBAL_DATA và DATA_LOCK)
try:
    from ros_comms.ros_handler import GLOBAL_DATA, DATA_LOCK,GLOBAL_PATHS_DATA
except ImportError:
    import threading
    print("WARN (MapViewer): Running in standalone/debug mode.")
    DATA_LOCK = threading.Lock()

    GLOBAL_DATA = {}
    GLOBAL_PATHS_DATA = {}
# --- UTILITY FUNCTIONS ---

def convert_pixel_to_metric(px, py, image_width_px, image_height_px, pixels_per_meter):
    # ... (giữ nguyên)
    if pixels_per_meter == 0:
        return 0, 0
    cx, cy = image_width_px / 2.0, image_height_px / 2.0
    x = (px - cx) / pixels_per_meter
    y = -(py - cy) / pixels_per_meter
    return x, y

def load_node_data(map_data_path, image_width_px, image_height_px, pixels_per_meter):
    # ... (giữ nguyên)
    try:
        with open(map_data_path, 'r', encoding='utf-8') as f:
            map_data = json.load(f)
        df = pd.DataFrame.from_dict(map_data['nodes'], orient='index', columns=['x_pixel', 'y_pixel'])
        df[['x', 'y']] = df.apply(
            lambda r: convert_pixel_to_metric(r['x_pixel'], r['y_pixel'], image_width_px, image_height_px, pixels_per_meter),
            axis=1, result_type='expand')
        return df
    except Exception as e:
        print(f"ERROR loading node data from '{map_data_path}': {e}")
        traceback.print_exc()
        return pd.DataFrame()

def get_image_dimensions(image_path):
    # ... (giữ nguyên)
    try:
        with Image.open(image_path) as img:
            return img.size
    except FileNotFoundError:
        print(f"ERROR: Background image not found at '{image_path}'")
        return None, None

# --- REGISTER CALLBACK ---
def register_callbacks(app, map_data_file_path, path_library_path, image_asset_path, pixels_per_meter=20.0):

    print("INFO (MapViewer): One-time static data setup...")

    # --- Load static image and map data ---
    width_px, height_px = get_image_dimensions(image_asset_path)
    if width_px is None:
        print("CRITICAL: Failed to load image, fallback size used.")
        width_px, height_px = 1000, 1000

    width_m, height_m = width_px / pixels_per_meter, height_px / pixels_per_meter

    nodes_df = load_node_data(map_data_file_path, width_px, height_px, pixels_per_meter)
    if nodes_df.empty:
        print("CRITICAL: Node data failed to load.")

    # Static traces created once
    static_nodes_trace = go.Scattergl(
        x=nodes_df['x'], y=nodes_df['y'], mode='markers',
        marker=dict(size=10, color='#1f77b4'),
        hovertext=[f"<b>Node: {i}</b><br>({r.x:.2f} m, {r.y:.2f} m)<extra></extra>"
                   for i, r in nodes_df.iterrows()],
        hoverinfo='text', name='Nodes') if not nodes_df.empty else None

    layout_image = None
    if width_px:
        layout_image = dict(
            source=app.get_asset_url(os.path.basename(image_asset_path)),
            xref="x", yref="y",
            x=-width_m / 2, y=height_m / 2,
            sizex=width_m, sizey=height_m,
            sizing="contain", opacity=0.5, layer="below")

    robot_colors = GLOBAL_DATA.get('robot_colors', {})
    
    print("INFO: Setup complete.")

    # --- CALLBACKS LẤY DỮ LIỆU ĐỘNG VÀ LƯU VÀO STORE ---
    # Các callback này chỉ nhiệm vụ lấy dữ liệu từ GLOBAL_DATA và lưu vào dcc.Store
    
    @app.callback(
        Output('poses-data-store', 'data'),
        Input('dynamic-update-interval', 'n_intervals')
    )
    def update_poses_store(_):
        # Lấy dữ liệu pose từ GLOBAL_DATA
        with DATA_LOCK:
            poses = GLOBAL_DATA.get('poses', {})
        return poses

    @app.callback(
        Output('lidar-data-store', 'data'),
        Input('dynamic-update-interval', 'n_intervals')
    )
    def update_lidar_store(_):
        # Lấy dữ liệu lidar từ GLOBAL_DATA
        with DATA_LOCK:
            lidar_data = GLOBAL_DATA.get('lidar', {})
        return lidar_data

    @app.callback(
        Output('paths-data-store', 'data'),
        Input('dynamic-update-interval', 'n_intervals')
    )
    def update_paths_store(_):
        # Lấy dữ liệu path thô
        with DATA_LOCK:
            paths = GLOBAL_PATHS_DATA
            
        return paths

    # --- CALLBACK HIỂN THỊ CHÍNH ---
    # Callback duy nhất này sẽ cập nhật biểu đồ, dựa trên các Store đã được cập nhật
    
    @app.callback(
        Output('map-graph', 'figure'),
        Input('layer-toggle-checklist', 'value'),
        Input('poses-data-store', 'data'),
        Input('lidar-data-store', 'data'),
        Input('paths-data-store', 'data')
    )
    def render_map_figure(selected_layers, poses, lidar_data, paths):
        fig = go.Figure()
        selected_layers = selected_layers if isinstance(selected_layers, list) else []

        # 1. Thêm layer tĩnh (Nodes)
        if 'nodes' in selected_layers:
            if static_nodes_trace:
                fig.add_trace(static_nodes_trace)
            else:
                return go.Figure().update_layout(title_text="Error: Could not load map data.", plot_bgcolor='#EAEAEA')

        # 2. Xử lý và thêm các layer động từ dữ liệu đã được lưu
        dynamic_traces = []
        poses = poses or {}
        lidar_data = lidar_data or {}
        paths = paths or {} # Dữ liệu này đã được đơn giản hóa

        # Thêm Lidar Traces
        if 'lidars' in selected_layers:
            for name, points in lidar_data.items():
                if points:
                    try:
                        x, y = zip(*points)
                        dynamic_traces.append(go.Scattergl(
                            x=x, y=y, mode='markers',
                            marker=dict(size=3, color=robot_colors.get(name, 'black'), opacity=0.6),
                            hoverinfo='none', name=f'{name}_lidar'))
                    except:
                        pass # Bỏ qua nếu dữ liệu không đúng định dạng

        # Thêm Robot Traces
        if 'robots' in selected_layers:
            for name, p in poses.items():
                if p and 'x' in p and 'y' in p:
                    hover = f"<b>Robot: {name}</b><br>({p['x']:.2f} m, {p['y']:.2f} m)<br>Status: Idle<extra></extra>"
                    dynamic_traces.append(go.Scattergl(
                        x=[p['x']], y=[p['y']], mode='markers',
                        marker=dict(size=20, color=robot_colors.get(name, 'black'), symbol='diamond',
                                    line=dict(width=2, color='black')),
                        hovertext=[hover], hoverinfo='text', name=name))

        # Thêm Path Traces (đã được đơn giản hóa)
        # if 'paths' in selected_layers:
        #     for name, path_points in paths.items():
        #         if path_points:
        #             path_x = [p[0] for p in path_points]
        #             path_y = [p[1] for p in path_points]
        #             color = robot_colors.get(name, 'gray')
        #             hover_text = [f"<b>{name} Path</b><br>({x:.2f} m, {y:.2f} m)" for x, y in zip(path_x, path_y)]
        #             dynamic_traces.append(go.Scattergl(
        #                 x=path_x, y=path_y, mode='lines+markers',
        #                 line=dict(width=2, dash='dot', color=color),
        #                 marker=dict(size=4, color=color),
        #                 hovertext=hover_text, hoverinfo='text', name=f"{name}_path"))
        
        if dynamic_traces:
            fig.add_traces(dynamic_traces)

        # 3. Cập nhật layout và phạm vi hiển thị
        padding = 5
        if not nodes_df.empty:
            x_rng = [nodes_df['x'].min() - padding, nodes_df['x'].max() + padding]
            y_rng = [nodes_df['y'].min() - padding, nodes_df['y'].max() + padding]
        else:
            x_rng, y_rng = [-10, 10], [-10, 10]

        fig.update_layout(
            xaxis=dict(range=x_rng, visible=False, scaleanchor="y", scaleratio=1),
            yaxis=dict(range=y_rng, visible=False),
            showlegend=False, margin=dict(l=0, r=0, t=0, b=0), plot_bgcolor='white',
            hovermode='closest', hoverlabel=dict(bgcolor="white", font_size=14)
        )

        # 4. Thêm ảnh nền (nếu được chọn)
        if 'map' in selected_layers and layout_image:
            fig.add_layout_image(layout_image)

        return fig