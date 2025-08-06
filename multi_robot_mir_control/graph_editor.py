# -*- coding: utf-8 -*-

"""
============================================================================
                        TRÌNH SOẠN THẢO ĐỒ THỊ GIAO THÔNG
============================================================================
Phiên bản hoàn chỉnh cuối cùng.
- Đã sửa lỗi vẽ cung tròn ngược bằng cách xử lý chính xác hệ tọa độ
  của Tkinter (Y đi xuống).
- Đã đồng bộ hóa logic vẽ trên giao diện và vẽ ra file ảnh.
- Tất cả các chức năng hoạt động đồng bộ và chính xác.
"""

import tkinter as tk
from tkinter import messagebox, simpledialog
from PIL import Image, ImageTk, ImageDraw, ImageFont
import json
import math
from scipy.spatial.distance import euclidean
import os

# --- LỚP DIALOG ĐỂ SỬA CẠNH ---
class EditEdgeDialog(tk.Toplevel):
    def __init__(self, parent, edge_data):
        super().__init__(parent)
        self.title("Sửa Cạnh"); self.transient(parent); self.grab_set()
        self.result = None; self.edge = edge_data.copy()
        frame = tk.Frame(self, padx=15, pady=10); frame.pack()
        self.direction_var = tk.StringVar(value=self.edge.get('direction', 'thuan'))
        tk.Label(frame, text="Hướng đi:").pack()
        tk.Radiobutton(frame, text="thuan", variable=self.direction_var, value="thuan").pack(anchor=tk.W)
        tk.Radiobutton(frame, text="nghich", variable=self.direction_var, value="nghich").pack(anchor=tk.W)
        btn_frame = tk.Frame(self, pady=5); btn_frame.pack()
        tk.Button(btn_frame, text="OK", command=self.on_ok, width=10).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Hủy", command=self.destroy, width=10).pack(side=tk.LEFT, padx=5)
    def on_ok(self):
        self.edge['direction'] = self.direction_var.get(); self.result = self.edge; self.destroy()

# --- LỚP CHÍNH ---
class MapEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Map Editor - Hoàn chỉnh")
        self.nodes, self.edges, self.history = {}, [], []
        self.mode = "normal"
        self.image, self.tk_image = None, None
        self.temp_node_pos, self.edge_start_node, self.arc_mid_pos = None, None, None
        self.canvas = tk.Canvas(root, width=800, height=600, bg='white')
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.right_frame = tk.Frame(root, width=250)
        self.right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=10, pady=5)
        self.right_frame.pack_propagate(False)
        self.setup_ui()

    def setup_ui(self):
        action_frame = tk.LabelFrame(self.right_frame, text="Hành động")
        action_frame.pack(fill=tk.X, pady=5)
        tk.Button(action_frame, text="Hoàn tác (Undo)", command=self.undo_last_action).pack(fill=tk.X)
        self.delete_mode_button = tk.Button(action_frame, text="Bật Chế độ Xóa", command=self.toggle_delete_mode, bg="light gray")
        self.delete_mode_button.pack(fill=tk.X, pady=(5,0))
        file_frame = tk.LabelFrame(self.right_frame, text="File Operations")
        file_frame.pack(fill=tk.X, pady=5)
        node_frame = tk.LabelFrame(self.right_frame, text="Node")
        node_frame.pack(fill=tk.X, pady=5)
        edge_frame = tk.LabelFrame(self.right_frame, text="Cạnh / Cung")
        edge_frame.pack(fill=tk.X, pady=5)
        self.status_var = tk.StringVar(value="Sẵn sàng.")
        self.status_bar = tk.Label(self.root, textvariable=self.status_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.bind("<Button-1>", self.on_click); self.canvas.bind("<Double-Button-1>", self.on_double_click); self.canvas.bind("<Motion>", self.on_mouse_move)
        tk.Label(file_frame, text="Ảnh nền:").pack(anchor='w'); self.image_filename_var = tk.StringVar(value="src/mir_robot/mir_control/Map.png"); tk.Entry(file_frame, textvariable=self.image_filename_var).pack(fill=tk.X, padx=5); tk.Button(file_frame, text="Tải ảnh nền", command=self.load_image).pack(fill=tk.X, padx=5, pady=(0,5))
        tk.Label(file_frame, text="File JSON:").pack(anchor='w'); self.json_filename_var = tk.StringVar(value="src/mir_robot/mir_control/data/map_data.json"); tk.Entry(file_frame, textvariable=self.json_filename_var).pack(fill=tk.X, padx=5); json_btn_frame = tk.Frame(file_frame); json_btn_frame.pack(fill=tk.X); tk.Button(json_btn_frame, text="Tải JSON", command=self.load_json).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(5,2)); tk.Button(json_btn_frame, text="Lưu JSON", command=self.save_json).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(2,5), pady=(0,5))
        tk.Label(file_frame, text="File ảnh đầu ra:").pack(anchor='w'); self.output_filename_var = tk.StringVar(value="/src/mir_robot/mir_control/map_output.png"); tk.Entry(file_frame, textvariable=self.output_filename_var).pack(fill=tk.X, padx=5); tk.Button(file_frame, text="Lưu ảnh map", command=self.save_map_as_image).pack(fill=tk.X, padx=5, pady=(0,5))
        tk.Label(node_frame, text="Tên node:").pack(); self.node_name_var = tk.StringVar(); tk.Entry(node_frame, textvariable=self.node_name_var).pack(fill=tk.X, padx=5); tk.Button(node_frame, text="Thêm node tại điểm đã chọn", command=self.add_node).pack(pady=5)
        self.edge_type_var = tk.StringVar(value="line"); tk.Label(edge_frame, text="Loại:").pack(); tk.Radiobutton(edge_frame, text="Đường thẳng", variable=self.edge_type_var, value="line", command=self._on_edge_type_change).pack(anchor=tk.W); tk.Radiobutton(edge_frame, text="Cung tròn (3 điểm)", variable=self.edge_type_var, value="arc", command=self._on_edge_type_change).pack(anchor=tk.W)
        self.direction_var = tk.StringVar(value="thuan"); tk.Label(edge_frame, text="Hướng đi (làn):").pack(pady=(5,0)); tk.Radiobutton(edge_frame, text="thuan", variable=self.direction_var, value="thuan").pack(anchor=tk.W); tk.Radiobutton(edge_frame, text="nghich", variable=self.direction_var, value="nghich").pack(anchor=tk.W)
        try: self.pil_font = ImageFont.truetype("arial.ttf", 12)
        except IOError: self.pil_font = ImageFont.load_default()
        self._on_edge_type_change()

    def on_click(self, event):
        pos = (event.x, event.y)
        if self.mode == 'delete':
            clicked_node = self.find_node_by_position(pos);
            if clicked_node: self.delete_node(clicked_node); return
            clicked_edge_info = self.find_edge_by_position(pos);
            if clicked_edge_info: self.delete_edge(clicked_edge_info[0]); return
            return
        edge_type = self.edge_type_var.get()
        clicked_node = self.find_node_by_position(pos)
        if not self.edge_start_node:
            if clicked_node:
                self.edge_start_node, self.temp_node_pos = clicked_node, None
                msg = "Click vào node kết thúc." if edge_type == 'line' else "Click vào điểm giữa của cung."
                self.update_status(f"Đã chọn node bắt đầu: {clicked_node}. {msg}")
            else:
                self.temp_node_pos = pos; self.update_status(f"Đã chọn vị trí {pos}. Nhập tên và thêm node.")
            self.redraw_map(); return
        if edge_type == 'line':
            if clicked_node and clicked_node != self.edge_start_node: self.create_edge(self.edge_start_node, clicked_node)
            else: self.update_status("Đã hủy tạo cạnh.")
            self.reset_creation_state()
        elif edge_type == 'arc':
            if not self.arc_mid_pos:
                self.arc_mid_pos = pos; self.update_status(f"Đã chọn điểm giữa {pos}. Click vào node kết thúc.")
            else:
                if clicked_node and clicked_node != self.edge_start_node: self.create_edge(self.edge_start_node, clicked_node, self.arc_mid_pos)
                else: self.update_status("Đã hủy tạo cung tròn.")
                self.reset_creation_state()
        self.redraw_map()

    def on_double_click(self, event):
        if self.mode == 'delete': return
        clicked_edge_info = self.find_edge_by_position((event.x, event.y))
        if clicked_edge_info:
            edge, index = clicked_edge_info
            dialog = EditEdgeDialog(self.root, edge)
            self.root.wait_window(dialog)
            if dialog.result:
                self.edges[index].update(dialog.result); self.update_status("Đã cập nhật hướng cạnh."); self.redraw_map()
            return
        clicked_node = self.find_node_by_position((event.x, event.y))
        if clicked_node: messagebox.showinfo("Sửa Node", f"Chức năng sửa node chưa được hỗ trợ.\nNode được chọn: {clicked_node}")

    def on_mouse_move(self, event):
        if self.edge_start_node and self.mode == 'normal':
            self.redraw_map()
            start_pos = self.nodes[self.edge_start_node]; end_pos = (event.x, event.y)
            if self.edge_type_var.get() == 'arc' and self.arc_mid_pos:
                 self.canvas.create_line(start_pos, self.arc_mid_pos, dash=(4, 2), fill='grey')
                 self.canvas.create_line(self.arc_mid_pos, end_pos, dash=(4, 2), fill='grey')
            else: self.canvas.create_line(start_pos, end_pos, dash=(4, 2), fill='grey')

    def add_node(self):
        name = self.node_name_var.get().strip()
        if not self.temp_node_pos: messagebox.showwarning("Thiếu vị trí", "Vui lòng click lên bản đồ."); return
        if not name: messagebox.showwarning("Thiếu tên", "Vui lòng nhập tên cho node."); return
        if name in self.nodes: messagebox.showwarning("Trùng tên", f"Tên node '{name}' đã tồn tại."); return
        self.nodes[name] = self.temp_node_pos
        self.history.append({'action': 'add_node', 'name': name})
        self.temp_node_pos = None; self.node_name_var.set("")
        self.redraw_map(); self.update_status(f"Đã tạo node: {name}")
    
    def create_edge(self, start_node, end_node, mid_pos=None):
        edge_type = 'arc' if mid_pos else 'line'
        new_edge = {'from': start_node, 'to': end_node, 'type': edge_type, 'direction': self.direction_var.get()}
        if edge_type == "line":
            new_edge['length'] = euclidean(self.nodes[start_node], self.nodes[end_node])
        else:
            p1, p2 = self.nodes[start_node], self.nodes[end_node]
            params = self._get_arc_params(p1, mid_pos, p2)
            if not params: messagebox.showerror("Lỗi hình học", "3 điểm thẳng hàng hoặc trùng nhau."); return
            new_edge.update(params)
        self.edges.append(new_edge)
        self.history.append({'action': 'add_edge', 'edge': new_edge})
        self.update_status(f"Đã tạo {edge_type} từ '{start_node}' đến '{end_node}'.")

    def undo_last_action(self):
        if not self.history: self.update_status("Không có gì để hoàn tác."); return
        last_action = self.history.pop()
        action_type = last_action['action']
        if action_type == 'add_node':
            del self.nodes[last_action['name']]; self.update_status(f"Hoàn tác: Đã xóa node '{last_action['name']}'.")
        elif action_type == 'add_edge':
            self.edges.pop(); self.update_status("Hoàn tác: Đã xóa cạnh vừa tạo.")
        elif action_type == 'delete_node':
            node_data = last_action['node_data']; self.nodes[node_data['name']] = node_data['coords']; self.edges.extend(last_action['deleted_edges']); self.update_status(f"Hoàn tác: Đã phục hồi node '{node_data['name']}'.")
        elif action_type == 'delete_edge':
            self.edges.insert(last_action['index'], last_action['edge']); self.update_status("Hoàn tác: Đã phục hồi cạnh.")
        self.redraw_map()
        
    def toggle_delete_mode(self):
        self.reset_creation_state()
        if self.mode == "normal": self.mode = "delete"; self.delete_mode_button.config(text="Tắt Chế độ Xóa", relief=tk.SUNKEN, bg="salmon"); self.update_status("Chế độ Xóa: Click để xóa."); self.canvas.config(cursor="X_cursor")
        else: self.mode = "normal"; self.delete_mode_button.config(text="Bật Chế độ Xóa", relief=tk.RAISED, bg="light gray"); self.update_status("Sẵn sàng."); self.canvas.config(cursor="")
        self.redraw_map()
        
    def delete_node(self, name_to_delete):
        if name_to_delete not in self.nodes: return
        deleted_node_data = {'name': name_to_delete, 'coords': self.nodes[name_to_delete]}; deleted_edges, remaining_edges = [], []
        for edge in self.edges:
            if edge['from'] == name_to_delete or edge['to'] == name_to_delete: deleted_edges.append(edge)
            else: remaining_edges.append(edge)
        self.edges = remaining_edges; del self.nodes[name_to_delete]
        self.history.append({'action': 'delete_node', 'node_data': deleted_node_data, 'deleted_edges': deleted_edges})
        self.update_status(f"Đã xóa node '{name_to_delete}' và các cạnh liên quan."); self.redraw_map()
        
    def delete_edge(self, edge_to_delete):
        if edge_to_delete not in self.edges: return
        index = self.edges.index(edge_to_delete); self.edges.remove(edge_to_delete)
        self.history.append({'action': 'delete_edge', 'edge': edge_to_delete, 'index': index})
        self.update_status(f"Đã xóa cạnh từ '{edge_to_delete['from']}' đến '{edge_to_delete['to']}'."); self.redraw_map()
        
    def load_image(self):
        path = self.image_filename_var.get()
        try: self.image = Image.open(path).convert("RGB"); self.canvas.config(width=self.image.width, height=self.image.height); self.redraw_map(); self.update_status(f"Đã tải ảnh nền: {os.path.basename(path)}")
        except Exception as e: messagebox.showerror("Lỗi", f"Không thể mở ảnh: {e}")
    
    def save_json(self):
        path = self.json_filename_var.get()
        if not path: messagebox.showerror("Lỗi", "Vui lòng nhập tên file JSON."); return
        edges_to_save = []
        for edge in self.edges:
            e = edge.copy()
            if 'center' in e: e['center'] = list(e['center'])
            if 'mid_point' in e: e['mid_point'] = list(e['mid_point'])
            edges_to_save.append(e)
        data = {'nodes': {n: list(c) for n,c in self.nodes.items()}, 'edges': edges_to_save}
        try:
            with open(path, 'w', encoding='utf-8') as f: json.dump(data, f, indent=4, ensure_ascii=False)
            messagebox.showinfo("Thành công", f"Đã lưu vào {path}"); self.update_status(f"Đã lưu dữ liệu vào {os.path.basename(path)}")
        except Exception as e: messagebox.showerror("Lỗi", f"Không thể lưu file JSON: {e}")
    
    def load_json(self):
        path = self.json_filename_var.get()
        if not path: return
        try:
            with open(path, 'r', encoding='utf-8') as f: data = json.load(f)
            self.nodes = {n: tuple(c) for n, c in data.get('nodes', {}).items()}
            self.edges = data.get('edges', [])
            for edge in self.edges:
                edge.setdefault('type', 'line'); edge.setdefault('direction', 'thuan')
                if edge['type'] == 'arc':
                    if 'center' in edge and isinstance(edge['center'], list): edge['center'] = tuple(edge['center'])
                    if 'mid_point' in edge and isinstance(edge['mid_point'], list): edge['mid_point'] = tuple(edge['mid_point'])
                    p1, p2, mid_point = self.nodes.get(edge['from']), self.nodes.get(edge['to']), edge.get('mid_point')
                    if p1 and p2 and mid_point:
                        params = self._get_arc_params(p1, mid_point, p2)
                        if params: edge.update(params)
                        else:
                            messagebox.showwarning("Cảnh báo tải JSON", f"Cung tròn từ {edge['from']} đến {edge['to']} không hợp lệ. Chuyển thành đường thẳng.")
                            edge['type'] = 'line'
                            for key in ['center', 'radius', 'mid_point', 'start_angle_rad', 'sweep_angle_rad']: edge.pop(key, None)
                            edge['length'] = euclidean(p1, p2)
                    else:
                        messagebox.showwarning("Cảnh báo tải JSON", f"Cung tròn từ {edge['from']} đến {edge['to']} thiếu thông tin. Chuyển thành đường thẳng.")
                        edge['type'] = 'line'
                        for key in ['center', 'radius', 'mid_point', 'start_angle_rad', 'sweep_angle_rad']: edge.pop(key, None)
                        edge['length'] = euclidean(p1, p2) if p1 and p2 else 0
            self.history.clear(); self.redraw_map(); self.update_status(f"Đã tải dữ liệu từ {os.path.basename(path)}")
        except Exception as e: messagebox.showerror("Lỗi", f"Không thể tải file JSON: {e}")
        
    def save_map_as_image(self):
        path = self.output_filename_var.get()
        if not path: messagebox.showerror("Lỗi", "Vui lòng nhập tên file ảnh đầu ra."); return
        self.update_status("Đang xuất ảnh...")
        output_image = self.image.copy() if self.image else Image.new('RGB', (self.canvas.winfo_width(), self.canvas.winfo_height()), 'white')
        draw = ImageDraw.Draw(output_image)
        for edge in self.edges:
            if edge['from'] not in self.nodes or edge['to'] not in self.nodes: continue
            color = 'blue' if edge['direction'] == 'thuan' else 'orange'
            if edge['type'] == 'line':
                p1, p2 = self.nodes[edge['from']], self.nodes[edge['to']]
                draw.line([p1, p2], fill=color, width=2); self._draw_arrow_pil(draw, p1, p2, color)
            elif edge['type'] == 'arc': self._draw_arc_pil(draw, edge, color)
        for name, (x, y) in self.nodes.items():
            r = 5; draw.ellipse((x-r, y-r, x+r, y+r), fill='red', outline='black'); draw.text((x, y-10), name, fill='black', font=self.pil_font, anchor='ms')
        try:
            output_image.save(path); messagebox.showinfo("Thành công", f"Đã lưu vào {path}"); self.update_status(f"Đã lưu bản đồ vào {os.path.basename(path)}")
        except Exception as e: messagebox.showerror("Lỗi", f"Không thể lưu ảnh: {e}")

    def _get_arc_params(self, p1, mid_point, p2):
        calc = self._calculate_circle_from_three_points(p1, mid_point, p2)
        if not calc: return None
        center, radius = calc
        # Sửa lỗi: Đảo dấu Y khi tính góc để phù hợp hệ tọa độ Descartes
        start_angle_rad = math.atan2(-(p1[1] - center[1]), p1[0] - center[0])
        mid_angle_rad = math.atan2(-(mid_point[1] - center[1]), mid_point[0] - center[0])
        end_angle_rad = math.atan2(-(p2[1] - center[1]), p2[0] - center[0])
        norm_end = end_angle_rad - start_angle_rad
        norm_mid = mid_angle_rad - start_angle_rad
        if norm_end < 0: norm_end += 2 * math.pi
        if norm_mid < 0: norm_mid += 2 * math.pi
        sweep_angle_rad = norm_end if norm_mid < norm_end else (norm_end - (2 * math.pi))
        return {
            "center": center, "radius": radius,
            "start_angle_rad": start_angle_rad, "sweep_angle_rad": sweep_angle_rad,
            "mid_point": mid_point, "length": abs(sweep_angle_rad * radius)
        }

    def draw_arc_on_canvas(self, edge, color):
        center, radius = edge['center'], edge['radius']
        start_angle_rad, sweep_angle_rad = edge['start_angle_rad'], edge['sweep_angle_rad']
        start_deg, extent_deg = math.degrees(start_angle_rad), math.degrees(sweep_angle_rad)
        bbox = (center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius)
        self.canvas.create_arc(bbox, start=start_deg, extent=extent_deg, style=tk.ARC, outline=color, width=2)
        end_angle_rad = start_angle_rad + sweep_angle_rad
        p2_x = center[0] + radius * math.cos(end_angle_rad)
        p2_y = center[1] - radius * math.sin(end_angle_rad) # Sửa lỗi: Đảo dấu Y
        p1_x_virt = center[0] + (radius - 1) * math.cos(end_angle_rad)
        p1_y_virt = center[1] - (radius - 1) * math.sin(end_angle_rad) # Sửa lỗi: Đảo dấu Y
        self.canvas.create_line((p1_x_virt, p1_y_virt), (p2_x, p2_y), fill=color, arrow=tk.LAST, arrowshape=(10, 12, 5))

    def _draw_arc_pil(self, draw, edge, color):
        center, radius = edge['center'], edge['radius']
        start_angle_rad, sweep_angle_rad = edge['start_angle_rad'], edge['sweep_angle_rad']
        start_deg, end_deg = math.degrees(start_angle_rad), math.degrees(start_angle_rad + sweep_angle_rad)
        # Pillow diễn giải góc ngược với Tkinter, nên ta chỉ cần đảo dấu start và end
        # so với hệ tọa độ toán học.
        pil_start_deg = -start_deg
        pil_end_deg = -end_deg
        if sweep_angle_rad > 0: # CCW trong toán học là CW trong Pillow
            pil_start_deg, pil_end_deg = pil_end_deg, pil_start_deg
        
        bbox = [center[0] - radius, center[1] - radius, center[0] + radius, center[1] + radius]
        draw.arc(bbox, start=pil_start_deg, end=pil_end_deg, fill=color, width=2)
        end_angle_rad = start_angle_rad + sweep_angle_rad
        p2_x = center[0] + radius * math.cos(end_angle_rad)
        p2_y = center[1] - radius * math.sin(end_angle_rad) # Sửa lỗi: Đảo dấu Y
        p1_x_virt = center[0] + (radius-1) * math.cos(end_angle_rad)
        p1_y_virt = center[1] - (radius-1) * math.sin(end_angle_rad) # Sửa lỗi: Đảo dấu Y
        self._draw_arrow_pil(draw, (p1_x_virt, p1_y_virt), (p2_x, p2_y), color)

    def _calculate_circle_from_three_points(self, p1, p2, p3):
        x1, y1 = p1; x2, y2 = p2; x3, y3 = p3
        D = 2 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))
        if abs(D) < 1e-8: return None
        Ux = ((x1**2 + y1**2) * (y2 - y3) + (x2**2 + y2**2) * (y3 - y1) + (x3**2 + y3**2) * (y1 - y2)) / D
        Uy = ((x1**2 + y1**2) * (x3 - x2) + (x2**2 + y2**2) * (x1 - x3) + (x3**2 + y3**2) * (x2 - x1)) / D
        center = (Ux, Uy); radius = euclidean(center, p1)
        return center, radius

    def reset_creation_state(self): self.edge_start_node, self.arc_mid_pos, self.temp_node_pos = None, None, None
    def update_status(self, message): self.status_var.set(message)
    def _on_edge_type_change(self): self.reset_creation_state(); self.redraw_map()
    def find_node_by_position(self, pos, tolerance=10):
        for name, (nx, ny) in self.nodes.items():
            if euclidean(pos, (nx, ny)) <= tolerance: return name
        return None
    def find_edge_by_position(self, pos, tolerance=5):
        for i, edge in enumerate(self.edges):
            if edge['from'] not in self.nodes or edge['to'] not in self.nodes: continue
            p1, p2 = self.nodes[edge['from']], self.nodes[edge['to']]
            if edge['type'] == 'line':
                if self.point_to_segment_dist(pos, p1, p2) < tolerance: return edge, i
            elif edge['type'] == 'arc':
                if 'center' in edge and abs(euclidean(pos, edge['center']) - edge['radius']) < tolerance: return edge, i
        return None
    def point_to_segment_dist(self, p, a, b):
        px, py = p; ax, ay = a; bx, by = b; l2 = (bx - ax)**2 + (by - ay)**2
        if l2 == 0: return euclidean(p, a)
        t = max(0, min(1, ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / l2))
        proj = (ax + t * (bx - ax), ay + t * (by - ay)); return euclidean(p, proj)
    def _draw_arrow_pil(self, draw, p1, p2, color):
        if p1 == p2: return
        x1, y1 = p1; x2, y2 = p2; arrow_length, arrow_angle = 10, math.radians(30); angle = math.atan2(y1 - y2, x1 - x2)
        p3_x, p3_y = x2 + arrow_length * math.cos(angle - arrow_angle), y2 + arrow_length * math.sin(angle - arrow_angle)
        p4_x, p4_y = x2 + arrow_length * math.cos(angle + arrow_angle), y2 + arrow_length * math.sin(angle + arrow_angle)
        draw.polygon([(x2, y2), (p3_x, p3_y), (p4_x, p4_y)], fill=color)
    def redraw_map(self):
        self.canvas.delete("all")
        if self.image: self.tk_image = ImageTk.PhotoImage(self.image); self.canvas.create_image(0, 0, anchor=tk.NW, image=self.tk_image)
        for edge in self.edges:
            if edge['from'] not in self.nodes or edge['to'] not in self.nodes: continue
            color = 'blue' if edge['direction'] == 'thuan' else 'orange'
            if edge['type'] == 'line':
                p1, p2 = self.nodes[edge['from']], self.nodes[edge['to']]
                self.canvas.create_line(p1, p2, fill=color, width=2, arrow=tk.LAST, arrowshape=(10, 12, 5))
            elif edge['type'] == 'arc': self.draw_arc_on_canvas(edge, color)
        for name, (x, y) in self.nodes.items():
            r = 5; fill_color = 'cyan' if self.edge_start_node == name else ('salmon' if self.mode=='delete' else 'red')
            self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=fill_color, outline='black')
            self.canvas.create_text(x, y-10, text=name, anchor=tk.S, font=("Arial", 10, "bold"))
        if self.temp_node_pos: x, y = self.temp_node_pos; r=5; self.canvas.create_oval(x-r, y-r, x+r, y+r, outline='green', dash=(4, 2), width=2)

if __name__ == '__main__':
    root = tk.Tk()
    app = MapEditor(root)
    root.mainloop()