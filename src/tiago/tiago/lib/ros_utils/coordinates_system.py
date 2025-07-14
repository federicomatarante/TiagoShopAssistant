def bottom_left_to_center(x, y, width, height):
    """
    Converts coordinates from bottom-left origin to center origin.

    Parameters:
    - x, y: Coordinates in bottom-left origin system.
    - width, height: Dimensions of the map/image.

    Returns:
    - (x_center, y_center): Coordinates in center origin system.
    """
    x_center = x - width / 2
    y_center = y - height / 2
    return x_center, y_center


def center_to_bottom_left(x, y, width, height):
    """
    Converts coordinates from center origin to bottom-left origin.

    Parameters:
    - x, y: Coordinates in center origin system.
    - width, height: Dimensions of the map/image.

    Returns:
    - (x_bl, y_bl): Coordinates in bottom-left origin system.
    """
    x_bl = x + width / 2
    y_bl = y + height / 2
    return x_bl, y_bl