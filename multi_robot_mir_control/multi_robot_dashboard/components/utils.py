METER_TO_PIXEL = 20

def meters_to_pixels_list(values, scale=METER_TO_PIXEL):
    return [int(v * scale) for v in values]

def pixels_to_meters(x_px, y_px, scale=METER_TO_PIXEL):
    """
    Chuyển từ pixel sang mét.
    """
    x_m = x_px / scale
    y_m = y_px / scale
    return x_m, y_m
