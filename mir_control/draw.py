import cv2
import numpy as np
import math

# Load ảnh nền
background = cv2.imread("/home/nguyen1/catkin_ws/src/mir_robot/mir_gazebo/maps/map_mir_2_595x386.jpg")  # Thay bằng ảnh của bạn
canvas = background.copy()

drawing_line = False
drawing_arc = False
line_start = None
arc_center = None

# Danh sách các hình đã vẽ
shapes = []  # ví dụ: {"type": "line", "start": (x1, y1), "end": (x2, y2)}

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

def draw(event, x, y, flags, param):
    global drawing_line, drawing_arc, line_start, arc_center, canvas, shapes

    if event == cv2.EVENT_LBUTTONDOWN:  # Chuột trái: đường thẳng
        if not drawing_line:
            line_start = (x, y)
            drawing_line = True
        else:
            line_end = (x, y)
            cv2.line(canvas, line_start, line_end, (0, 255, 0), 2)
            shapes.append({
                "type": "line",
                "start": line_start,
                "end": line_end
            })
            drawing_line = False

    elif event == cv2.EVENT_RBUTTONDOWN:  # Chuột phải: cung tròn
        if not drawing_arc:
            arc_center = (x, y)
            drawing_arc = True
        else:
            arc_point = (x, y)
            radius = int(distance(arc_center, arc_point))
            # Cung tròn từ góc 0 đến 180 độ
            cv2.ellipse(canvas, arc_center, (radius, radius), 0, 0, 180, (255, 0, 0), 2)
            shapes.append({
                "type": "arc",
                "center": arc_center,
                "radius": radius,
                "start_angle": 0,
                "end_angle": 180
            })
            drawing_arc = False

# Tạo cửa sổ và gán callback
cv2.namedWindow("Draw Shapes")
cv2.setMouseCallback("Draw Shapes", draw)

while True:
    cv2.imshow("Draw Shapes", canvas)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
    elif key == ord("s"):
        print("Shapes:")
        for s in shapes:
            print(s)

cv2.destroyAllWindows()
