from dataclasses import dataclass
from typing import List
import math


@dataclass
class Point2D:
    x: int
    y: int

    def distance(self, other: "Point2D") -> float:
        dx = self.x - other.x
        dy = self.y - other.y
        return math.hypot(dx, dy)

    def to_json(self) -> dict:
        return {"x": self.x, "y": self.y}


@dataclass
class Polygon:
    vertices: List[Point2D]

    def contains_point(self, point: Point2D) -> bool:
        """
        Determine whether the given point lies inside the polygon using the ray casting algorithm.
        """
        n = len(self.vertices)
        inside = False
        x, y = point.x, point.y

        for i in range(n):
            j = (i - 1) % n
            xi, yi = self.vertices[i].x, self.vertices[i].y
            xj, yj = self.vertices[j].x, self.vertices[j].y

            intersects = ((yi > y) != (yj > y)) and \
                         (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi)
            if intersects:
                inside = not inside

        return inside
