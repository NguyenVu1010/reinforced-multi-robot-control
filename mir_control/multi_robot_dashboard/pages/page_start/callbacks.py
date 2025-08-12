# pages/page_start/callbacks.py

from dash import Input, Output, State, html
import dash_bootstrap_components as dbc

def register_callbacks(app, ros_handler):
    """
    Đăng ký các callback chỉ dành riêng cho trang Start.
    - app: Đối tượng Dash app.
    - ros_handler: Instance của lớp RosHandler để có thể gửi lệnh.
    """

    @app.callback(
        # Output('start-page-status', 'children', allow_duplicate=True),
        Output('launch-ros-button', 'disabled'),
        Input('launch-ros-button', 'n_clicks'),
        prevent_initial_call=True
    )
    def handle_launch_button(n_clicks):
        """
        Xử lý sự kiện khi nút Launch được nhấn.
        Gửi lệnh 'LAUNCH' vào hàng đợi của RosHandler.
        """
        # Gửi lệnh vào hàng đợi để luồng RosHandler xử lý
        print("INFO: Launch button clicked. Sending LAUNCH command to handler.")
        ros_handler.command_queue.put('LAUNCH')
        
        # Cập nhật trạng thái ngay lập tức và vô hiệu hóa nút
        # status_message = html.Div([
        #     dbc.Spinner(size="sm", className="me-2"),
        #     "Launch command sent. Initializing..."
        # ])
        
        return True # Vô hiệu hóa nút để tránh nhấn nhiều lần

    @app.callback(
        Output('start-page-status', 'children'),
        Input('app-status-store', 'data')
    )
    def update_status_display(data):
        """
        Cập nhật thông báo trạng thái trên trang Start.
        """
        data = data or {}
        message = data.get('message', "Waiting for command...")
        launched = data.get('launched', False)
        
        spinner_component = ""
        if launched and not data.get('connected', False):
            # <<< THAY ĐỔI CHÍNH Ở ĐÂY >>>
            # Bọc Spinner trong một Span và áp dụng className cho Span
            spinner_component = html.Span(
                dbc.Spinner(size="sm", color="primary"),
                className="me-2" # Áp dụng margin-end 2 cho Span
            )
        
        return html.Div([spinner_component, f" {message}"])