# pages/start_page/layout.py 
# (Hoặc bạn có thể đặt nó trong layouts/ nếu muốn)

from dash import html
import dash_bootstrap_components as dbc


layout = dbc.Container(
        dbc.Row(
            dbc.Col(
                dbc.Card(
                    dbc.CardBody([
                        html.H2("ROS Control Center", className="card-title text-center"),
                        html.P("Khởi động và kết nối đến hệ thống ROS", className="text-center text-muted"),
                        html.Hr(),
                        
                        # Vùng hiển thị trạng thái
                        html.Div(id='start-page-status', className="text-center mb-4", children=[
                            html.Span("Sẵn sàng để khởi động.")
                        ]),
                        
                        # Nút bấm
                        dbc.Button(
                            "LAUNCH & CONNECT",
                            id="launch-ros-button",
                            color="primary",
                            size="lg",
                            className="w-100",
                            n_clicks=0
                        )
                    ]),
                    className="shadow-sm mt-5",
                    style={"maxWidth": "500px"}
                ),
                width="auto"
            )
        ),
        className="d-flex justify-content-center align-items-center",
        style={"height": "80vh"}
    )