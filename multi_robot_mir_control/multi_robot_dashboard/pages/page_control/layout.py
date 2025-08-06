# pages/page_control/layout.py

from dash import html, dcc
import dash_bootstrap_components as dbc
from components.map_viewer.layout import create_layout as create_map_viewer_component
from components.control_panel.layout import create_layout as create_control_panel_component

# Định nghĩa layout như một biến tĩnh
layout = html.Div([
    dbc.Row(
        [
            dbc.Col(create_map_viewer_component(), md=8),
            dbc.Col(create_control_panel_component(), md=4),
        ],
        className="align-items-stretch",
        style={'height': '85vh'}
    ),
    
    # Interval này chỉ dùng để kích hoạt các callback cập nhật DỮ LIỆU
    # Nó được tạo ra một lần cùng với layout này và không bao giờ bị render lại.
    dcc.Interval(
        id='page-control-interval',
        interval=1000,
        n_intervals=0
    )
])