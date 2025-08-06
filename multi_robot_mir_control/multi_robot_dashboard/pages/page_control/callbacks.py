# pages/page_control/callbacks.py

# Import các module callback của component
from components.map_viewer import callbacks as map_viewer_callbacks
from components.control_panel import callbacks as control_panel_callbacks

def register_callbacks(app, map_data_path, path_library_path, image_asset_path, pixels_per_meter=20.0):
    """
    Đăng ký callbacks cho trang control.
    Nó nhận các đường dẫn file từ app.py và chuyển tiếp chúng.
    """
    
    # Chuyển tiếp các tham số đến map_viewer
    map_viewer_callbacks.register_callbacks(
        app, 
        map_data_file_path=map_data_path, 
        path_library_path=path_library_path, 
        image_asset_path=image_asset_path,
        pixels_per_meter=pixels_per_meter  
    )
    
    # Đăng ký callback cho control_panel (không cần tham số file)
    control_panel_callbacks.register_callbacks(app)
    
    print("INFO: Callbacks for Page Control have been registered.")