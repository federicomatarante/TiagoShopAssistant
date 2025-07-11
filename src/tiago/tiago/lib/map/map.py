import datetime
import json
import threading
from typing import List, Dict, Tuple, Optional

from PIL import Image

from tiago.lib.map.geometry import Point2D, Polygon
from tiago.lib.map.map_display import MapDisplay


class Observations:
    """
    A class representing all the dynamic observations done in a map.
    Attributes:
        _last_seen_products (Dict[str, Tuple[Point2D, datetime]]): A dictionary mapping product IDs to their last known position (Point2D) and the timestamp when they were last seen.
        _last_seen_staff (Dict[str, Tuple[Point2D, datetime]]): A dictionary mapping staff IDs to their last known position (Point2D) and the timestamp when they were last seen.
    """
    _last_seen_products: Dict[str, Tuple[Point2D, datetime.datetime]] = {}
    _last_seen_staff: Dict[str, Tuple[Point2D, datetime.datetime]] = {}

    def clear_expired(self, max_age: datetime.timedelta) -> None:
        """
        Removes products and staff that have not been seen for longer than the specified maximum age.

        :param max_age: The maximum allowed time since the entity was last seen.
        """
        current_time = datetime.datetime.now()

        outdated_products = [product_id for product_id, (position, timestamp) in self._last_seen_products.items()
                             if current_time - timestamp > max_age]

        for product_id in outdated_products:
            del self._last_seen_products[product_id]

        outdated_staff = [staff_id for staff_id, (position, timestamp) in self._last_seen_staff.items()
                          if current_time - timestamp > max_age]

        for staff_id in outdated_staff:
            del self._last_seen_staff[staff_id]

    def update_product_location(self, product_id: str, position: Point2D) -> None:
        """
        Updates the last seen location and timestamp for a specific product.

        :param product_id: The unique identifier of the product.
        :param position: The last known position of the product (Point2D).
        """
        timestamp = datetime.datetime.now()

        self._last_seen_products[product_id] = (position, timestamp)

    def update_staff_location(self, staff_id: str, position: Point2D) -> None:
        """
        Updates the last seen location and timestamp for a specific staff member.

        :param staff_id: The unique identifier of the staff member.
        :param position: The last known position of the staff member (Point2D).
        :param timestamp: The timestamp when the staff member was last seen.
        """
        timestamp = datetime.datetime.now()
        self._last_seen_staff[staff_id] = (position, timestamp)

    def get_product_location(self, product_id: str) -> Optional[Point2D]:
        """
        Retrieves the last known location and timestamp for a product.

        :param product_id: The unique identifier of the product.
        :return: A tuple containing the product's last known position (Point2D) and the timestamp.
        :raises KeyError: If the product ID is not found.
        """
        try:
            return self._last_seen_products[product_id][0]
        except KeyError:
            return None

    def get_staff_location(self, staff_id: str) -> Optional[Point2D]:
        """
        Retrieves the last known location and timestamp for a staff member.

        :param staff_id: The unique identifier of the staff member.
        :return: A tuple containing the staff member's last known position (Point2D) and the timestamp.
        :raises KeyError: If the staff ID is not found.
        """
        try:
            return self._last_seen_staff[staff_id][0]
        except KeyError:
            return None




class Map:
    """
    A class representing a map with an occupancy grid, width, height, and tracking of areas, products, and staff.

    Attributes:
        occupancy_grid (List[List[int]]): A 2D grid representing the occupancy status of each cell. 100 means occupied, 0 means free.
        width (int): The width of the map in terms of number of cells.
        height (int): The height of the map in terms of number of cells.
        _named_areas (Dict[str, Polygon]): A dictionary mapping area names to corresponding polygons defining the areas.
        observations (Observations): the dynamic observations done in the map.
    """

    occupancy_grid: List[List[int]]
    width: int
    height: int
    observations: Observations
    _named_areas: Dict[str, Polygon]

    def __init__(self, width: int, height: int, occupancy_grid: List[List[int]] = None, path:str = 'ground.mp'):
        """
        Initializes a new map with the given dimensions and an empty occupancy grid.

        :param width: The width of the map in terms of number of cells.
        :param height: The height of the map in terms of number of cells.
        :param occupancy_grid: The occupancy grid of the map. Optional.
        """
        self.width = width
        self.height = height
        self.observations = Observations()
        self.occupancy_grid = occupancy_grid if occupancy_grid is not None else [[0 for _ in range(width)] for _ in
                                                                                 range(height)]
        self._named_areas = {}
        self._last_seen_products = {}
        self._last_seen_staff = {}
        self.path = path

    @property
    def area_names(self) -> List[str]:
        """
        Returns a list of names of all the areas defined in the map.

        :return: A list of area names.
        """
        return list(self._named_areas.keys())

    def get_area_polygon(self, name: str) -> Optional[Polygon]:
        """
        Retrieves the polygon representing the area with the given name.

        :param name: The name of the area.
        :return: The Polygon object representing the area, or None if the area does not exist.
        """
        return self._named_areas.get(name, None)

    def add_area(self, name: str, area: Polygon) -> None:
        """
        Adds a named area to the map.

        :param name: The name of the area.
        :param area: A Polygon object representing the area on the map.
        """
        for vertex in area.vertices:
            if not (0 <= vertex.x < self.width and 0 <= vertex.y < self.height):
                raise ValueError("Vertex coordinates out of bounds.")
        self._named_areas[name] = area

    def get_area(self, point: Point2D) -> Optional[Tuple[str, Polygon]]:
        """
        Finds the area that contains the given point.

        :param point: The point (of type Point2D) to check against the areas.
        :return: The Area object that contains the point, or None if no area contains the point.
        """
        for name, area in self._named_areas.items():
            if area.contains_point(point):
                return (name, area)
        return None

    def is_occupied(self, x: int, y: int) -> bool:
        """
        Checks if a given cell in the occupancy grid is occupied.

        :param x: The x-coordinate (column) of the cell.
        :param y: The y-coordinate (row) of the cell.
        :return: True if the cell is occupied, False otherwise.
        :raises ValueError: If the coordinates are outside the bounds of the map.
        """
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError("Coordinates out of bounds.")
        return self.occupancy_grid[y][x] == 100

    def set_cell_occupancy(self, x: int, y: int, occupied: bool) -> None:
        """
        Sets the occupancy state of a given cell in the occupancy grid.

        :param x: The x-coordinate (column) of the cell.
        :param y: The y-coordinate (row) of the cell.
        :param occupied: A boolean indicating whether the cell is occupied (True) or free (False).
        :raises ValueError: If the coordinates are outside the bounds of the map.
        """
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError("Coordinates out of bounds.")
        self.occupancy_grid[y][x] = 100 if occupied else 0

    @staticmethod
    def from_image(image_path: str, max_value: int = 128) -> "Map":
        """
        Initializes the map from an image, setting the occupancy grid based on the image's pixel data.

        :param image_path: The path to the image representing the map.
        :param max_value: The maximum value of a pixel tot be considered an obstacle.
        """
        img = Image.open(image_path).convert('L')  # 'L' mode is for grayscale
        img_width, img_height = img.size
        occupancy_grid = []
        for y in range(img_height):
            row = []
            for x in range(img_width):
                pixel_value = img.getpixel((x, y))
                row.append(100 if pixel_value < max_value else 0)
            occupancy_grid.append(row)
        return Map(img_width, img_height, occupancy_grid)

    def save(self):
        def point_to_dict(p: Point2D) -> Dict:
            return {"x": p.x, "y": p.y}

        def polygon_to_dict(poly: Polygon) -> Dict:
            return {"vertices": [point_to_dict(v) for v in poly.vertices]}

        def datetime_to_str(dt: datetime.datetime) -> str:
            return dt.isoformat()

        data = {
            "width": self.width,
            "height": self.height,
            "occupancy_grid": self.occupancy_grid,
            "named_areas": {name: polygon_to_dict(area) for name, area in self._named_areas.items()},
            "observations": {
                "products": {
                    pid: {
                        "position": point_to_dict(pos),
                        "timestamp": datetime_to_str(ts)
                    } for pid, (pos, ts) in self.observations._last_seen_products.items()
                },
                "staff": {
                    sid: {
                        "position": point_to_dict(pos),
                        "timestamp": datetime_to_str(ts)
                    } for sid, (pos, ts) in self.observations._last_seen_staff.items()
                }
            }
        }

        with open(self.path, 'w') as f:
            json.dump(data, f)

    @staticmethod
    def from_file(path: str) -> "Map":
        def dict_to_point(d: Dict) -> Point2D:
            return Point2D(float(d["x"]), float(d["y"]))

        def dict_to_polygon(d: Dict) -> Polygon:
            return Polygon([dict_to_point(p) for p in d["vertices"]])

        def str_to_datetime(s: str) -> datetime.datetime:
            return datetime.datetime.fromisoformat(s)

        with open(path, 'r') as f:
            data = json.load(f)

        m = Map(data["width"], data["height"], data["occupancy_grid"])

        for name, poly_data in data.get("named_areas", {}).items():
            m._named_areas[name] = dict_to_polygon(poly_data)

        for pid, item in data.get("observations", {}).get("products", {}).items():
            pos = dict_to_point(item["position"])
            ts = str_to_datetime(item["timestamp"])
            m.observations._last_seen_products[pid] = (pos, ts)

        for sid, item in data.get("observations", {}).get("staff", {}).items():
            pos = dict_to_point(item["position"])
            ts = str_to_datetime(item["timestamp"])
            m.observations._last_seen_staff[sid] = (pos, ts)

        return m

    def display(self, interactive=True):
        """
        Displays a 2D occupancy grid as an image and overlays named areas.
        If interactive=True, allows clicking on the map to get coordinates and shows action buttons.
        This now runs in a separate thread.

        :param interactive: Whether to enable interactive clicking, defaults to True
        """

        def run_display():
            map_display = MapDisplay(self)
            map_display.display(interactive)

        thread = threading.Thread(target=run_display, daemon=True)
        thread.start()
        return thread  # Optionally return the thread in case you want to join or check status
