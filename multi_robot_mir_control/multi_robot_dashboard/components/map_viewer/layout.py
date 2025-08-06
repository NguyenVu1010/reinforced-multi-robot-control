# components/map_viewer/layout.py

from dash import dcc, html
import dash_bootstrap_components as dbc
import plotly.graph_objects as go

def create_selection_details_card():
    """Tạo card để hiển thị thông tin chi tiết (nếu bạn muốn dùng lại sau này)."""
    return dbc.Card(
        dbc.CardBody([
            html.Div([
                html.I(className="fas fa-mouse-pointer", style={'marginRight': '10px'}),
                html.H5("Selection Details", className="card-title"),
            ], style={'display': 'flex', 'alignItems': 'center'}),
            html.Hr(),
            html.Div(
                id='selection-details-output',
                children="Hover on a map component to see details."
            )
        ]),
        style={'height': '100%', 'display': 'flex', 'flexDirection': 'column', 'overflowY': 'auto'}
    )

def create_map_viewer_card():
    """
    Tạo card chứa bản đồ tương tác và các nút điều khiển layer.
    """
    map_card_style = {
        'backgroundColor': 'white', 'borderRadius': '16px', 'padding': '24px',
        'boxShadow': '0 8px 32px rgba(0, 0, 0, 0.08)', 'border': '1px solid #f0f0f0',
        'marginBottom': '24px', 'height': '100%',
        'display': 'flex', 'flexDirection': 'column',
        'position': 'relative'
    }

    # Tạo một figure ban đầu trống để tránh lỗi "None"
    initial_fig = go.Figure()
    initial_fig.update_layout(
        plot_bgcolor='#EAEAEA',
        xaxis={'visible': False},
        yaxis={'visible': False},
        annotations=[{
            "text": "Loading Map Data...",
            "xref": "paper",
            "yref": "paper",
            "showarrow": False,
            "font": {"size": 16}
        }]
    )

    return html.Div([
        # Tiêu đề card
        html.Div([
            html.I(className="fas fa-map-marked-alt", style={'color': '#1abc9c', 'fontSize': '20px', 'marginRight': '12px'}),
            html.H4("Map Viewer", style={'color': '#2c3e50', 'fontWeight': '600', 'margin': '0'})
        ], style={'display': 'flex', 'alignItems': 'center', 'marginBottom': '20px'}),

        # Vùng chứa biểu đồ
        html.Div(
            dcc.Graph(
                id='map-graph',
                figure=initial_fig,
                config={'displayModeBar': False},
                style={'width': '100%', 'height': '100%'}
            ),
            style={
                'position': 'absolute',
                'top': '70px', 'left': '24px', 'right': '24px', 'bottom': '80px',
            }
        ),

        # Phần điều khiển layer
        html.Div([
            html.Hr(),
            html.H6("Toggle Layers", className="mb-3"),
            dbc.Checklist(
                options=[
                    {"label": "🗺️ Map", "value": "map"},
                    {"label": "📍 Nodes", "value": "nodes"},
                    {"label": "↔️ Paths", "value": "paths"},
                    {"label": "🤖 Robots", "value": "robots"},
                ],
                value=["map", "nodes", "paths", "robots"],
                id="layer-toggle-checklist",
                inline=True,
                switch=True,
            ),
            dcc.Interval(
                id='dynamic-update-interval', 
                interval=200,  # 200ms = 5Hz. Bạn có thể điều chỉnh giá trị này.
                n_intervals=0
            ),
        ], style={'marginTop': 'auto'})
    ], style=map_card_style)

# Hàm "lắp ráp" chính cho toàn bộ component map_viewer
def create_layout():
    return dbc.Row([
        # dbc.Col(create_map_viewer_card(), md=9),
        # dbc.Col(create_selection_details_card(), md=3)
        # Tạm thời chỉ hiển thị map card để tối giản
        dbc.Col(create_map_viewer_card(), width=12)
    ], className="align-items-stretch", style={'height': '100%'})