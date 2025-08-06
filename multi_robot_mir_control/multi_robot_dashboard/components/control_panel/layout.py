# multi_robot_dashboard/components/control_panel/layout.py

from dash import dcc, html
import dash_bootstrap_components as dbc

def create_mission_modal():
    """Cửa sổ pop-up (Modal) để tạo một mẫu nhiệm vụ mới."""
    return dbc.Modal(
        [
            dbc.ModalHeader(dbc.ModalTitle("Create New Mission Template")),
            dbc.ModalBody(
                dbc.Form([
                    dbc.Row([
                        dbc.Label("Task Type", width=4),
                        dbc.Col(
                            dcc.Dropdown(
                                id='cp-task-type-dropdown',
                                options=[
                                    {'label': 'Go to a Point (GOTO)', 'value': 'GOTO'},
                                    {'label': 'Deliver between 2 Points (DELIVERY)', 'value': 'DELIVERY'}
                                ],
                                value='GOTO',
                                clearable=False
                            ), width=8
                        ),
                    ], className="mb-3"),
                    html.Div(id='cp-task-form-inputs')
                ])
            ),
            dbc.ModalFooter([
                dbc.Button("Close", id="cp-close-modal-button", color="secondary"),
                dbc.Button("Save Template", id="cp-submit-mission-button", color="primary")
            ]),
        ],
        id="cp-create-mission-modal",
        is_open=False,
    )

def create_available_missions_card():
    """Tạo card Available Missions (Mission Templates)."""
    card_style = {'backgroundColor': 'white', 'borderRadius': '16px', 'padding': '24px', 'boxShadow': '0 8px 32px rgba(0, 0, 0, 0.08)', 'border': '1px solid #f0f0f0', 'marginBottom': '24px'}
    table_container_style = {"maxHeight": "400px", "overflowY": "auto", "border": "1px solid #e9ecef", "borderRadius": "12px"}
    table_header_style = {'backgroundColor': '#f8f9fa', 'borderBottom': '2px solid #e9ecef', 'position': 'sticky', 'top': '0', 'zIndex': '10'}
    return html.Div([
        html.Div([
            html.I(className="fas fa-list-alt me-2", style={'color': '#27ae60'}),
            html.H4("Mission Templates", style={'color': '#2c3e50', 'fontWeight': '600', 'margin': '0'})
        ], style={'display': 'flex', 'alignItems': 'center', 'marginBottom': '20px'}),
        html.Div(
            html.Table([
                html.Thead(html.Tr([
                    html.Th("Template Name", style={'padding': '12px'}),
                    html.Th("Type", style={'padding': '12px'}),
                    html.Th("Actions", style={'padding': '12px', 'textAlign': 'center'})
                ], style=table_header_style)),
                html.Tbody(id="mission-templates-tbody")
            ], className="table table-hover", style={'margin': '0'}),
            style=table_container_style
        )
    ], style=card_style)

def create_active_queue_card():
    """Tạo card Active Queue và các nút hành động chính."""
    primary_button_style = {'padding': '12px 24px', 'fontSize': '14px', 'fontWeight': '600', 'borderRadius': '10px', 'border': 'none', 'color': 'white', 'cursor': 'pointer'}
    card_style = {'backgroundColor': 'white', 'borderRadius': '16px', 'padding': '24px', 'boxShadow': '0 8px 32px rgba(0, 0, 0, 0.08)', 'border': '1px solid #f0f0f0', 'marginBottom': '24px'}
    table_container_style = {"maxHeight": "400px", "overflowY": "auto", "border": "1px solid #e9ecef", "borderRadius": "12px"}
    table_header_style = {'backgroundColor': '#f8f9fa', 'borderBottom': '2px solid #e9ecef', 'position': 'sticky', 'top': '0', 'zIndex': '10'}
    return html.Div([
        html.Div([
            html.I(className="fas fa-tasks me-2", style={'color': '#e74c3c'}),
            html.H4("Active Execution Queue", style={'color': '#2c3e50', 'fontWeight': '600', 'margin': '0'})
        ], style={'display': 'flex', 'alignItems': 'center', 'marginBottom': '20px'}),
        html.Div(
            html.Table([
                html.Thead(html.Tr([
                    html.Th("Task ID", style={'padding': '12px'}),
                    html.Th("Name (Robot)", style={'padding': '12px'}),
                    html.Th("Status", style={'padding': '12px'}),
                    html.Th("Actions", style={'padding': '12px', 'textAlign': 'center'})
                ], style=table_header_style)),
                html.Tbody(id="task-queue-tbody")
            ], className="table table-hover", style={'margin': '0'}),
            style=table_container_style
        ),
        html.Div(id="cp-general-alert", style={"marginTop": "20px"}),
    ], style=card_style)

def create_main_actions_card():
    """Tạo card chứa các nút hành động chính."""
    primary_button_style = {'padding': '12px 24px', 'fontSize': '14px', 'fontWeight': '600', 'borderRadius': '10px', 'border': 'none', 'color': 'white', 'cursor': 'pointer'}
    card_style = {'backgroundColor': 'white', 'borderRadius': '16px', 'padding': '24px', 'boxShadow': '0 8px 32px rgba(0, 0, 0, 0.08)', 'border': '1px solid #f0f0f0'}
    return html.Div([
        dbc.Row([
            dbc.Col(dbc.Button([html.I(className="fas fa-plus me-2"), "Create Template"], id="cp-open-create-modal-btn", color="primary", style={**primary_button_style, 'width': '100%', 'background': 'linear-gradient(135deg, #007bff 0%, #0056b3 100%)'})),
            dbc.Col(dbc.Button([html.I(className="fas fa-stop me-2"), "Stop All Tasks"], id="cp-stop-all-btn", color="danger", style={**primary_button_style, 'backgroundColor': '#c0392b', 'width': '100%'})),
        ], justify="center", className="g-3")
    ], style=card_style)


def create_layout():
    """Tạo layout hoàn chỉnh và tối ưu cho Control Panel."""
    return html.Div([
        dbc.Row(dbc.Col(create_available_missions_card(), width=12)),
        dbc.Row(dbc.Col(create_active_queue_card(), width=12)),
        dbc.Row(dbc.Col(create_main_actions_card(), width=12)),
        
        # Các component ẩn
        create_mission_modal(),
        dcc.Interval(id='cp-interval-component', interval=2 * 1000, n_intervals=0),
        dcc.Store(id='cp-data-trigger-store'), # Store để trigger cập nhật giao diện
    ])