# components/sidebar.py

from dash import html
import dash_bootstrap_components as dbc

def create_sidebar():
    """Tạo component Sidebar."""
    
    sidebar = html.Div(
        [
            html.H2("Robot Dashboard", className="display-6"),
            html.Hr(),
            dbc.Nav(
                [
                    dbc.NavLink(
                        html.Div([html.I(className="fas fa-satellite-dish me-2"), "Live Control"]),
                        href="/", 
                        active="exact"
                    ),
                    dbc.NavLink(
                        html.Div([html.I(className="fas fa-edit me-2"), "Map Editor"]),
                        href="/map-editor", 
                        active="exact"
                    ),
                ],
                vertical=True,
                pills=True,
            ),
        ],
        className="sidebar",
    )
    
    return sidebar