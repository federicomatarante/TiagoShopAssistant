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

    @property
    def centroid(self) -> Point2D:
        """
        Calculate the centroid of the polygon using the formula:
        Cx = (x1 + x2 + ... + xn) / n
        Cy = (y1 + y2 + ... + yn) / n
        where n is the number of vertices.
        """
        if not self.vertices:
            return Point2D(0, 0)

        x_sum = sum(vertex.x for vertex in self.vertices)
        y_sum = sum(vertex.y for vertex in self.vertices)
        n = len(self.vertices)

        return Point2D(x_sum // n, y_sum // n)

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
