import numpy as np
from typing import NamedTuple, Tuple


Ray = NamedTuple(
    "Ray",
    [
        ('origin', Tuple[int, int]),
        ('angle', float),
        ('dist', int),
    ])


class LocationTracker:
    """Keeps track of current location of the car"""
    pass


class Mapper:
    """Maps the environment"""

    EMPTY = 0
    UNKNOWN = 1
    FILLED = 2

    def __init__(self, size=100, dist_cutoff=8, connect_cutoff=5):
        self.size = size
        self.dist_cutoff = dist_cutoff
        self.connect_cutoff = connect_cutoff
        self.data = np.ones((size, size)) * Mapper.UNKNOWN
        self.rays = []

    def add_ray(self, ray: Ray) -> None:
        x, y = np.array(ray.origin).round().astype(int)
        self.data[y, x] = 0
        if self.rays:
            last_ray = self.rays[-1]
            last_ray_end = (last_ray.origin[0] + np.cos(last_ray.angle) * last_ray.dist,
                            last_ray.origin[1] + np.sin(last_ray.angle) * last_ray.dist)
            this_ray_end = (ray.origin[0] + np.cos(ray.angle) * ray.dist,
                            ray.origin[1] + np.sin(ray.angle) * ray.dist)
            self.shade_triangle(last_ray.origin, ray.origin, last_ray_end)
            self.shade_triangle(last_ray.origin, ray.origin, this_ray_end)
            end_point_dist = np.linalg.norm(
                    np.array(last_ray_end) - np.array(this_ray_end)) # type: ignore
            if end_point_dist < self.connect_cutoff:
                self.shade_triangle(
                    last_ray.origin, this_ray_end, last_ray_end)
                if max(ray.dist, last_ray.dist) < self.dist_cutoff:
                    print(last_ray_end)
                    self.draw_line(this_ray_end, last_ray_end)
        self.rays.append(ray)

    def shade_triangle(self, p1, p2, p3) -> None:
        """Clear the area inside the triangle defined by the three points"""
        p1 = np.array(p1)
        p2 = np.array(p2)
        p3 = np.array(p3)

        # Calculate the bounding box
        min_x = np.floor(min(p1[0], p2[0], p3[0])).astype(int)
        max_x = np.ceil(max(p1[0], p2[0], p3[0])).astype(int)
        min_y = np.floor(min(p1[1], p2[1], p3[1])).astype(int)
        max_y = np.ceil(max(p1[1], p2[1], p3[1])).astype(int)

        # Check each point in the bounding box
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                # Check if the point is inside the triangle
                inside = self.is_inside_triangle(p1, p2, p3, (x, y))
                if inside:
                    self.data[y, x] = Mapper.EMPTY

    def is_inside_triangle(self, p1, p2, p3, p) -> bool:
        """Check if a point is inside a triangle"""
        v0 = p3 - p1
        v1 = p2 - p1
        v2 = p - p1

        dot00 = np.dot(v0, v0)
        dot01 = np.dot(v0, v1)
        dot02 = np.dot(v0, v2)
        dot11 = np.dot(v1, v1)
        dot12 = np.dot(v1, v2)

        inv_denom = 1 / (dot00 * dot11 - dot01 * dot01)
        u = (dot11 * dot02 - dot01 * dot12) * inv_denom
        v = (dot00 * dot12 - dot01 * dot02) * inv_denom

        return (u >= 0) and (v >= 0) and (u + v < 1)

    def draw_line(self, p1, p2) -> None:
        """Draw a line between two points"""
        p1 = np.array(p1)
        p2 = np.array(p2)

        # Calculate the bounding box
        min_x = np.floor(min(p1[0], p2[0])).astype(int)
        max_x = np.ceil(max(p1[0], p2[0])).astype(int)
        min_y = np.floor(min(p1[1], p2[1])).astype(int)
        max_y = np.ceil(max(p1[1], p2[1])).astype(int)

        # Check each point in the bounding box
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                # Check if the point is on the line
                if self.is_on_line(p1, p2, (x, y)):
                    self.data[y, x] = Mapper.FILLED

    def is_on_line(self, p1, p2, p) -> bool:
        """Check if a point is on a line"""
        return np.linalg.norm(np.cross(p2 - p1, p1 - p)) / np.linalg.norm(p2 - p1) < 0.5

    def __str__(self) -> str:
        s = ''
        for row in self.data:
            for block in row:
                if block == Mapper.EMPTY:
                    s += ' '
                elif block == Mapper.UNKNOWN:
                    s += '▓'
                elif block == Mapper.FILLED:
                    s += '█'
            s += '\n'
        return s

    def plot(self, show=True, save_file=None) -> None:
        import matplotlib.pyplot as plt

        plt.figure(dpi=400)
        plt.pcolormesh(self.data, cmap='Greys', edgecolors='k', linewidth=1)
        for ray in self.rays:
            x, y = ray.origin
            dx = np.cos(ray.angle) * ray.dist
            dy = np.sin(ray.angle) * ray.dist
            plt.arrow(x + 0.5, y + 0.5, dx, dy, color='g', width=0.01,
                      head_width=0.05, head_length=0.02)

        plt.gca().set_aspect('equal')  # type: ignore

        if save_file and isinstance(save_file, str):
            plt.savefig(save_file, dpi=300)
        if show:
            plt.show()


if __name__ == '__main__':
    mapper = Mapper(25)
    mapper.add_ray(Ray((4, 4), np.radians(80), 10))
    mapper.add_ray(Ray((4, 4), np.radians(70), 10))
    mapper.add_ray(Ray((5, 5), np.radians(50), 10))
    mapper.add_ray(Ray((5, 5), np.radians(40), 10))
    mapper.add_ray(Ray((6, 6), np.radians(20), 6))
    mapper.add_ray(Ray((6, 6), np.radians(10), 6))
    mapper.add_ray(Ray((7, 7), np.radians(-10), 4))
    mapper.add_ray(Ray((7, 7), np.radians(-20), 4))
    mapper.add_ray(Ray((8, 8), np.radians(20), 5))
    mapper.add_ray(Ray((8, 8), np.radians(10), 4))
    mapper.plot()
