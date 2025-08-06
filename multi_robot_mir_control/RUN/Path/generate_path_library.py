# file: preprocess_graph.py
import json, math, numpy as np, os

class GraphPreprocessor:
    def __init__(self, input_file='src/mir_robot/mir_control/data/map_data.json', output_file='path_library.json'):
        self.input_file, self.output_file = input_file, output_file
        self.nodes, self.edges = {}, []

    def process(self, point_density=10):
        if not os.path.exists(self.input_file): print(f"Lỗi: Không tìm thấy file '{self.input_file}'"); return
        print(f"Đang đọc file '{self.input_file}'...")
        with open(self.input_file, 'r', encoding='utf-8') as f: data = json.load(f)
        self.nodes = {name: tuple(coords) for name, coords in data['nodes'].items()}
        self.edges = data['edges']
        print(f"Đã tải {len(self.nodes)} nodes và {len(self.edges)} cạnh.")
        
        processed_edges = []
        for i, edge in enumerate(self.edges):
            print(f"  - Đang xử lý cạnh {i+1}/{len(self.edges)}: {edge['from']} -> {edge['to']}")
            p1, p2 = self.nodes.get(edge['from']), self.nodes.get(edge['to'])
            if not p1 or not p2: print(f"    Cảnh báo: Bỏ qua cạnh."); continue
            
            path_points = []
            if edge['type'] == 'line': path_points = self._interpolate_line(p1, p2, point_density)
            elif edge['type'] == 'arc' and all(k in edge for k in ['start_angle_rad', 'sweep_angle_rad']):
                path_points = self._interpolate_arc(edge, point_density)
            else:
                print(f"    Cảnh báo: Cung tròn thiếu thông tin, xử lý như đường thẳng.")
                path_points = self._interpolate_line(p1, p2, point_density)
            
            processed_edge = edge.copy()
            processed_edge['path_points'] = path_points
            processed_edges.append(processed_edge)
        
        # <<< THAY ĐỔI: Chỉ lưu một danh sách 'edges' duy nhất >>>
        output_data = {"nodes": self.nodes, "edges": processed_edges}
        
        print(f"\nĐang lưu kết quả vào '{self.output_file}'...")
        with open(self.output_file, 'w', encoding='utf-8') as f: json.dump(output_data, f, indent=4)
        print("Hoàn tất!")

    def _interpolate_line(self, p1, p2, density):
        p1_np, p2_np = np.array(p1), np.array(p2); length = np.linalg.norm(p2_np - p1_np)
        num_points = max(2, int(length * density)); points = np.linspace(p1_np, p2_np, num_points)
        return [p.tolist() for p in points]
    def _interpolate_arc(self, edge, density):
        center, radius = np.array(edge['center']), edge['radius']
        start_angle, sweep_angle = edge['start_angle_rad'], edge['sweep_angle_rad']
        arc_length = abs(sweep_angle * radius); num_points = max(2, int(arc_length * density)); points = []
        for i in range(num_points):
            fraction = i / (num_points - 1); angle = start_angle + sweep_angle * fraction
            x = center[0] + radius * math.cos(angle); y = center[1] - radius * math.sin(angle)
            points.append([x, y])
        return points

if __name__ == '__main__':
    preprocessor = GraphPreprocessor(input_file='src/mir_robot/mir_control/data/map_data.json', output_file='src/mir_robot/mir_control/data/path_library.json')
    preprocessor.process(point_density=10)