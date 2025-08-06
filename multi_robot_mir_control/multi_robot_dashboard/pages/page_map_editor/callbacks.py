# callbacks.py
from dash import Input, Output, State, no_update, callback_context, dcc
import plotly.graph_objects as go
from PIL import Image, ImageDraw, ImageFont
import json
import math
from scipy.spatial.distance import euclidean
import os
import base64
import io

# XÓA CÁC HÀM HELPER TÍNH TOÁN Ở ĐÂY VÌ ĐÃ CHUYỂN SANG layout.py
# CHỈ GIỮ LẠI CÁC HÀM CẦN THIẾT
def point_to_segment_dist(p, a, b):
    px, py = p; ax, ay = a; bx, by = b; l2 = (bx - ax)**2 + (by - ay)**2
    if l2 == 0: return euclidean(p, a)
    t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2))
    proj = (ax + t * (bx - ax), ay + t * (by - ay)); return euclidean(p, proj)

try:
    pil_font = ImageFont.truetype("arial.ttf", 14, encoding="unic")
except IOError:
    pil_font = ImageFont.load_default()

def _draw_arrow_pil(draw, p1, p2, color):
    if p1 == p2: return
    x1, y1 = p1; x2, y2 = p2
    angle = math.atan2(y2 - y1, x2 - x1)
    arrow_length, arrow_angle = 12, math.radians(30)
    p3_x = x2 - arrow_length * math.cos(angle - arrow_angle)
    p3_y = y2 - arrow_length * math.sin(angle - arrow_angle)
    p4_x = x2 - arrow_length * math.cos(angle + arrow_angle)
    p4_y = y2 - arrow_length * math.sin(angle + arrow_angle)
    draw.polygon([(x2, y2), (p3_x, p3_y), (p4_x, p4_y)], fill=color)

def _draw_arc_pil(draw, edge, color):
    center, radius = edge['center'], edge['radius']
    start_angle_rad, sweep_angle_rad = edge['start_angle_rad'], edge['sweep_angle_rad']
    start_deg, end_deg = math.degrees(start_angle_rad), math.degrees(start_angle_rad + sweep_angle_rad)
    pil_start_deg, pil_end_deg = -start_deg, -end_deg
    if pil_start_deg > pil_end_deg: pil_start_deg, pil_end_deg = pil_end_deg, pil_start_deg
    bbox = [center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius]
    draw.arc(bbox, start=pil_start_deg, end=pil_end_deg, fill=color, width=2)
    end_angle_rad = start_angle_rad + sweep_angle_rad
    p_end = (center[0] + radius * math.cos(end_angle_rad), center[1] - radius * math.sin(end_angle_rad))
    p_before = (center[0] + (radius-1) * math.cos(end_angle_rad), center[1] - (radius-1) * math.sin(end_angle_rad))
    _draw_arrow_pil(draw, p_before, p_end, color)


# --- HÀM ĐĂNG KÝ CALLBACKS ---
def register_callbacks(app):

    # XÓA BỎ TOÀN BỘ CALLBACK initialize_graph_data Ở ĐÂY
    
    # --- CALLBACK VẼ LẠI BẢN ĐỒ ---
    # Callback này giữ nguyên
    @app.callback(
        Output('map-canvas', 'figure'),
        Input('graph-data-store', 'data'),
        Input('ui-state-store', 'data'),
        Input('image-store', 'data')
    )
    def redraw_map(graph_data, ui_state, image_data):
        nodes, edges = graph_data.get('nodes',{}), graph_data.get('edges',[])
        mode, temp_node_pos, edge_start_node = ui_state.get('mode', 'normal'), ui_state.get('temp_node_pos'), ui_state.get('edge_start_node')
        fig = go.Figure()

        fig.update_layout(
            xaxis=dict(range=[0, image_data['width']], showgrid=False, zeroline=False, visible=False),
            yaxis=dict(range=[image_data['height'], 0], showgrid=False, zeroline=False, visible=False, scaleanchor="x", scaleratio=1),
            showlegend=False, margin=dict(l=0, r=0, t=0, b=0), uirevision='constant'
        )
        if image_data.get('contents'):
            fig.add_layout_image(dict(source=image_data['contents'], xref="x", yref="y", x=0, y=0, sizex=image_data['width'], sizey=image_data['height'], sizing="stretch", layer="below"))

        shapes = []
        for edge in edges:
            if edge['from'] not in nodes or edge['to'] not in nodes: continue
            color = 'blue' if edge.get('direction', 'thuan') == 'thuan' else 'orange'
            p1, p2 = nodes[edge['from']], nodes[edge['to']]

            if edge.get('type') == 'line':
                fig.add_trace(go.Scatter(x=[p1[0], p2[0]], y=[p1[1], p2[1]], mode='lines', line=dict(color=color, width=2), hoverinfo='none'))
                fig.add_annotation(x=p2[0], y=p2[1], ax=p1[0], ay=p1[1], xref='x', yref='y', axref='x', ayref='y', showarrow=True, arrowhead=2, arrowsize=1.5, arrowwidth=1.5, arrowcolor=color)
            elif edge.get('type') == 'arc' and 'center' in edge:
                center, radius = edge['center'], edge['radius']
                start_rad, sweep_rad = edge['start_angle_rad'], edge['sweep_angle_rad']
                end_rad = start_rad + sweep_rad
                p_start = (center[0] + radius * math.cos(start_rad), center[1] - radius * math.sin(start_rad))
                p_end = (center[0] + radius * math.cos(end_rad), center[1] - radius * math.sin(end_rad))
                large_arc_flag = 1 if abs(sweep_rad) > math.pi else 0
                sweep_flag = 1 if sweep_rad > 0 else 0
                path = f"M {p_start[0]},{p_start[1]} A {radius},{radius} 0 {large_arc_flag},{sweep_flag} {p_end[0]},{p_end[1]}"
                shapes.append(dict(type='path', path=path, line_color=color, line_width=2))
                prev_rad = end_rad - 0.01 * (1 if sweep_rad > 0 else -1)
                p_prev = (center[0] + radius * math.cos(prev_rad), center[1] - radius * math.sin(prev_rad))
                fig.add_annotation(x=p_end[0], y=p_end[1], ax=p_prev[0], ay=p_prev[1], xref='x', yref='y', axref='x', ayref='y', showarrow=True, arrowhead=2, arrowsize=1.5, arrowwidth=1.5, arrowcolor=color)

        fig.update_layout(shapes=shapes)

        node_x, node_y, node_text, node_color = [], [], [], []
        for name, (x, y) in nodes.items():
            node_x.append(x); node_y.append(y); node_text.append(name)
            node_color.append('cyan' if edge_start_node == name else ('salmon' if mode == 'delete' else 'red'))
        fig.add_trace(go.Scatter(x=node_x, y=node_y, text=node_text, mode='markers+text', textposition='top center', marker=dict(size=10, color=node_color, line=dict(width=1, color='black')), textfont=dict(size=12, color='black'), hoverinfo='text', name='nodes'))

        if temp_node_pos:
            fig.add_shape(type="circle", x0=temp_node_pos[0]-5, y0=temp_node_pos[1]-5, x1=temp_node_pos[0]+5, y1=temp_node_pos[1]+5, line_color="green", line_width=2, line_dash="dash")
        return fig
    
    # BÂY GIỜ CHÚNG TA CẦN THÊM LẠI LOGIC TÍNH TOÁN CUNG TRÒN VÀO NƠI TẠO CẠNH
    # Tức là trong callback handle_map_clicks
    
    # Import lại các hàm tính toán vào đây
    from .layout import _get_arc_params
    
    @app.callback(
        Output('graph-data-store', 'data', allow_duplicate=True),
        Output('ui-state-store', 'data', allow_duplicate=True),
        Output('history-store', 'data', allow_duplicate=True),
        Output('status-bar', 'children', allow_duplicate=True),
        Output('edit-edge-modal', 'is_open'),
        Output('edit-direction-radio', 'value'),
        Output('edit-edge-info-store', 'data'),
        Input('map-canvas', 'clickData'),
        Input('map-canvas', 'doubleClickData'),
        State('ui-state-store', 'data'),
        State('graph-data-store', 'data'),
        State('history-store', 'data'),
        State('edge-type-radio', 'value'),
        State('direction-radio', 'value'),
        prevent_initial_call=True
    )
    def handle_map_clicks(clickData, dblClickData, ui_state, graph_data, history, edge_type, direction):
        # ... logic này giữ nguyên ...
        triggered_id = callback_context.triggered[0]['prop_id']
        nodes, edges = graph_data['nodes'], graph_data['edges']

        def find_node_by_position(pos, tolerance=10):
            return next((name for name, (nx, ny) in nodes.items() if euclidean(pos, (nx, ny)) <= tolerance), None)

        def find_edge_by_position(pos, tolerance=5):
            for i, edge in enumerate(edges):
                if edge['from'] not in nodes or edge['to'] not in nodes: continue
                p1, p2 = nodes[edge['from']], nodes[edge['to']]
                if edge['type'] == 'line' and point_to_segment_dist(pos, p1, p2) < tolerance: return edge, i
                if edge['type'] == 'arc' and 'center' in edge and abs(euclidean(pos, edge['center']) - edge['radius']) < tolerance: return edge, i
            return None

        if 'doubleClickData' in triggered_id and dblClickData and ui_state['mode'] != 'delete':
            pos = (dblClickData['points'][0]['x'], dblClickData['points'][0]['y'])
            clicked_edge_info = find_edge_by_position(pos)
            if clicked_edge_info:
                edge, index = clicked_edge_info
                return no_update, no_update, no_update, f"Đang sửa cạnh từ '{edge['from']}' đến '{edge['to']}'", True, edge.get('direction', 'thuan'), {'index': index}
            return no_update

        if 'clickData' in triggered_id and clickData:
            pos = (round(clickData['points'][0]['x']), round(clickData['points'][0]['y']))
            clicked_node = find_node_by_position(pos)
            if ui_state['mode'] == 'delete':
                if clicked_node:
                    deleted_node_data = {'name': clicked_node, 'coords': nodes[clicked_node]}
                    deleted_edges = [e for e in edges if e['from'] == clicked_node or e['to'] == clicked_node]
                    remaining_edges = [e for e in edges if e not in deleted_edges]
                    del nodes[clicked_node]
                    history.append({'action': 'delete_node', 'node_data': deleted_node_data, 'deleted_edges': deleted_edges})
                    return {'nodes': nodes, 'edges': remaining_edges}, ui_state, history, f"Đã xóa node '{clicked_node}'.", False, no_update, no_update
                clicked_edge_info = find_edge_by_position(pos)
                if clicked_edge_info:
                    edge_to_delete, index = clicked_edge_info
                    edges.pop(index)
                    history.append({'action': 'delete_edge', 'edge': edge_to_delete, 'index': index})
                    return {'nodes': nodes, 'edges': edges}, ui_state, history, f"Đã xóa cạnh.", False, no_update, no_update
                return no_update
            
            if not ui_state['edge_start_node']:
                if clicked_node:
                    ui_state.update({'edge_start_node': clicked_node, 'temp_node_pos': None})
                    msg = "Click node kết thúc." if edge_type == 'line' else "Click điểm giữa cung."
                    status = f"Đã chọn node bắt đầu: {clicked_node}. {msg}"
                else:
                    ui_state['temp_node_pos'] = pos
                    status = f"Đã chọn vị trí {pos}. Nhập tên và thêm node."
                return no_update, ui_state, no_update, status, False, no_update, no_update

            if edge_type == 'line':
                if clicked_node and clicked_node != ui_state['edge_start_node']:
                    start_node, end_node = ui_state['edge_start_node'], clicked_node
                    new_edge = {'from': start_node, 'to': end_node, 'type': 'line', 'direction': direction, 'length': euclidean(nodes[start_node], nodes[end_node])}
                    edges.append(new_edge)
                    history.append({'action': 'add_edge', 'edge': new_edge})
                    status = f"Đã tạo cạnh từ '{start_node}' đến '{end_node}'."
                else: status = "Đã hủy tạo cạnh."
                ui_state.update({'edge_start_node': None, 'arc_mid_pos': None})
                return {'nodes': nodes, 'edges': edges}, ui_state, history, status, False, no_update, no_update
            elif edge_type == 'arc':
                if not ui_state['arc_mid_pos']:
                    ui_state['arc_mid_pos'] = pos
                    return no_update, ui_state, no_update, f"Đã chọn điểm giữa {pos}. Click node kết thúc.", False, no_update, no_update
                else:
                    if clicked_node and clicked_node != ui_state['edge_start_node']:
                        start_node, mid_pos, end_node = ui_state['edge_start_node'], ui_state['arc_mid_pos'], clicked_node
                        params = _get_arc_params(nodes[start_node], mid_pos, nodes[end_node])
                        if not params: status = "Lỗi: 3 điểm thẳng hàng. Hủy tạo cung."
                        else:
                            new_edge = {'from': start_node, 'to': end_node, 'type': 'arc', 'direction': direction, **params}
                            edges.append(new_edge); history.append({'action': 'add_edge', 'edge': new_edge})
                            status = f"Đã tạo cung từ '{start_node}' đến '{end_node}'."
                    else: status = "Đã hủy tạo cung."
                    ui_state.update({'edge_start_node': None, 'arc_mid_pos': None})
                    return {'nodes': nodes, 'edges': edges}, ui_state, history, status, False, no_update, no_update
        return no_update
    
    # ... các callback còn lại giữ nguyên ...
    @app.callback(
        Output('graph-data-store', 'data', allow_duplicate=True), Output('ui-state-store', 'data', allow_duplicate=True), Output('history-store', 'data', allow_duplicate=True),
        Output('status-bar', 'children', allow_duplicate=True), Output('delete-mode-button', 'children'), Output('delete-mode-button', 'color'), Output('node-name-input', 'value'),
        Input('add-node-button', 'n_clicks'), Input('undo-button', 'n_clicks'), Input('delete-mode-button', 'n_clicks'), Input('edge-type-radio', 'value'),
        State('node-name-input', 'value'), State('ui-state-store', 'data'), State('graph-data-store', 'data'), State('history-store', 'data'),
        prevent_initial_call=True
    )
    def handle_actions(add_clicks, undo_clicks, delete_clicks, edge_type_change, node_name, ui_state, graph_data, history):
        triggered_id = callback_context.triggered[0]['prop_id'].split('.')[0]
        nodes, edges = graph_data['nodes'], graph_data['edges']
        status, delete_text, delete_color, new_node_name = no_update, no_update, no_update, node_name
        
        if triggered_id == 'edge-type-radio':
            ui_state.update({'temp_node_pos': None, 'edge_start_node': None, 'arc_mid_pos': None})
            return no_update, ui_state, no_update, "Đã đổi loại cạnh. Sẵn sàng tạo mới.", no_update, no_update, ""

        if triggered_id == 'add-node-button' and node_name and ui_state['temp_node_pos']:
            if node_name in nodes: status = f"Lỗi: Tên node '{node_name}' đã tồn tại."
            else:
                nodes[node_name] = ui_state['temp_node_pos']; history.append({'action': 'add_node', 'name': node_name})
                ui_state['temp_node_pos'] = None; status = f"Đã tạo node: {node_name}"; new_node_name = ""
        
        elif triggered_id == 'undo-button' and history:
            last_action = history.pop()
            action_type = last_action['action']
            if action_type == 'add_node': del nodes[last_action['name']]; status = f"Hoàn tác: Xóa node '{last_action['name']}'."
            elif action_type == 'add_edge': edges.pop(); status = "Hoàn tác: Xóa cạnh."
            elif action_type == 'delete_node': node_data = last_action['node_data']; nodes[node_data['name']] = node_data['coords']; edges.extend(last_action['deleted_edges']); status = f"Hoàn tác: Phục hồi node '{node_data['name']}'."
            elif action_type == 'delete_edge': edges.insert(last_action['index'], last_action['edge']); status = "Hoàn tác: Phục hồi cạnh."
        
        elif triggered_id == 'delete-mode-button':
            if ui_state['mode'] == 'normal': ui_state.update({'mode': 'delete', 'temp_node_pos': None, 'edge_start_node': None, 'arc_mid_pos': None}); status = "Chế độ Xóa: BẬT."; delete_text = "Tắt Chế độ Xóa"; delete_color = "danger"
            else: ui_state['mode'] = 'normal'; status = "Chế độ Xóa: TẮT."; delete_text = "Bật Chế độ Xóa"; delete_color = "secondary"

        return {'nodes': nodes, 'edges': edges}, ui_state, history, status, delete_text, delete_color, new_node_name
    
    @app.callback(Output("download-json", "data"), Input("save-json-button", "n_clicks"), State("graph-data-store", "data"), State("json-filename-input", "value"), prevent_initial_call=True)
    def save_json(n_clicks, graph_data, filename):
        data_to_save = {'nodes': graph_data['nodes'], 'edges': []}
        for edge in graph_data['edges']:
            e = {k: v for k, v in edge.items() if k in ['from', 'to', 'type', 'direction', 'mid_point']}
            data_to_save['edges'].append(e)
        return dict(content=json.dumps(data_to_save, indent=4, ensure_ascii=False), filename=filename or "map_data.json")

    @app.callback(Output("download-image", "data"), Input("save-image-button", "n_clicks"), State("graph-data-store", "data"), State("image-store", "data"), State("output-filename-input", "value"), prevent_initial_call=True)
    def save_map_as_image(n_clicks, graph_data, image_data, filename):
        nodes, edges = graph_data['nodes'], graph_data['edges']
        if image_data.get('contents'):
            decoded = base64.b64decode(image_data['contents'].split(',')[1])
            output_image = Image.open(io.BytesIO(decoded)).convert("RGB")
        else: output_image = Image.new('RGB', (image_data['width'], image_data['height']), 'white')
        
        draw = ImageDraw.Draw(output_image)
        for edge in edges:
            if edge['from'] not in nodes or edge['to'] not in nodes: continue
            color = 'blue' if edge.get('direction', 'thuan') == 'thuan' else 'orange'
            if edge['type'] == 'line': _draw_arrow_pil(draw, nodes[edge['from']], nodes[edge['to']], color); draw.line([nodes[edge['from']], nodes[edge['to']]], fill=color, width=2)
            elif edge['type'] == 'arc' and 'center' in edge: _draw_arc_pil(draw, edge, color)
        for name, (x, y) in nodes.items():
            r = 5; draw.ellipse((x-r, y-r, x+r, y+r), fill='red', outline='black'); draw.text((x, y-10), name, fill='black', font=pil_font, anchor='ms')
        
        img_byte_arr = io.BytesIO(); output_image.save(img_byte_arr, format='PNG')
        content = "data:image/png;base64," + base64.b64encode(img_byte_arr.getvalue()).decode()
        return dict(content=content, filename=filename or "map_output.png")
    
    @app.callback(
        Output('graph-data-store', 'data', allow_duplicate=True), Output('status-bar', 'children', allow_duplicate=True), Output('edit-edge-modal', 'is_open', allow_duplicate=True),
        Input('edit-edge-ok-button', 'n_clicks'), State('edit-direction-radio', 'value'), State('edit-edge-info-store', 'data'), State('graph-data-store', 'data'),
        prevent_initial_call=True
    )
    def handle_edit_edge_ok(n_clicks, new_direction, edit_info, graph_data):
        if not edit_info: return no_update
        index = edit_info.get('index')
        if index is not None and index < len(graph_data['edges']):
            graph_data['edges'][index]['direction'] = new_direction
            return graph_data, "Đã cập nhật hướng cạnh.", False
        return no_update, "Lỗi: Không tìm thấy cạnh để sửa.", False

    @app.callback(Output('edit-edge-modal', 'is_open', allow_duplicate=True), Input('edit-edge-cancel-button', 'n_clicks'), prevent_initial_call=True)
    def handle_edit_edge_cancel(n_clicks):
        return False