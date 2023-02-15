from typing import Union, List, NamedTuple, Tuple

import numpy as np
from scipy.ndimage.filters import gaussian_filter

from astar import astar


Ray = NamedTuple(
    "Ray",
    [
        ('origin', Tuple[int, int]),
        ('angle', float),
        ('dist', int),
    ])


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
                np.array(last_ray_end) - np.array(this_ray_end))  # type: ignore
            if end_point_dist < self.connect_cutoff:
                self.shade_triangle(
                    last_ray.origin, this_ray_end, last_ray_end)
                if max(ray.dist, last_ray.dist) < self.dist_cutoff:
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

    def plot(self, path=Union[None, List[Tuple[int, int]]],
             show=True, save_file=None) -> None:
        import matplotlib.pyplot as plt

        plt.figure(dpi=400)
        plt.pcolormesh(self.data, cmap='Greys')
        for ray in self.rays[-5:]:
            x, y = ray.origin
            dx = np.cos(ray.angle) * ray.dist
            dy = np.sin(ray.angle) * ray.dist
            plt.arrow(x + 0.5, y + 0.5, dx, dy, color='g', width=0.01,
                      head_width=0.05, head_length=0.02)

        if path:
            overlay = np.zeros_like(self.data)
            for x, y in path:
                overlay[y, x] = 1
            plt.pcolormesh(overlay, cmap='Reds', alpha=0.5)

        plt.gca().set_aspect('equal')  # type: ignore

        if save_file and isinstance(save_file, str):
            try:
                plt.savefig(save_file, dpi=300)
            except Exception as e:
                print('Failed to save plot', e)
        if show:
            plt.show()

    def route(self, start: Tuple[int, int], dest: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Find a route from start to dest"""
        obstacle_map = (self.data == self.FILLED).astype(float)
        blurred = gaussian_filter(obstacle_map, sigma=1)
        extruded = (blurred > 0.001).astype(int)
        import matplotlib.pyplot as plt
        plt.pcolormesh(extruded, cmap='Greys')
        plt.show()
        return astar(extruded.T, start, dest)


if __name__ == '__main__':
    # rays = [
    #     Ray(origin=(15, 10), angle=1.2566370614359172, dist=31),
    #     Ray(origin=(15, 10), angle=0.9424777960769379, dist=28),
    #     Ray(origin=(15, 10), angle=0.6283185307179586, dist=36),
    #     Ray(origin=(15, 10), angle=0.3141592653589793, dist=35),
    #     Ray(origin=(15, 10), angle=0.0, dist=42),
    #     Ray(origin=(15, 10), angle=0.3141592653589793, dist=50),
    #     Ray(origin=(15, 10), angle=0.6283185307179586, dist=50),
    #     Ray(origin=(15, 10), angle=0.9424777960769379, dist=38),
    #     Ray(origin=(15, 10), angle=1.2566370614359172, dist=35),
    #     Ray(origin=(15, 10), angle=1.5707963267948966, dist=32),
    #     Ray(origin=(15, 10), angle=1.8849555921538759, dist=29),
    #     Ray(origin=(15, 10), angle=2.199114857512855, dist=32),
    #     Ray(origin=(15, 10), angle=2.5132741228718345, dist=33),
    #     Ray(origin=(15, 10), angle=2.827433388230814, dist=29),
    #     Ray(origin=(15, 10), angle=3.141592653589793, dist=33),
    #     Ray(origin=(15, 10), angle=2.827433388230814, dist=39),
    #     Ray(origin=(15, 12), angle=2.5132741228718345, dist=50),
    #     Ray(origin=(15, 14), angle=2.199114857512855, dist=29),
    #     Ray(origin=(15, 16), angle=1.8849555921538759, dist=27),
    #     Ray(origin=(15, 17), angle=1.5707963267948966, dist=27),
    #     Ray(origin=(15, 18), angle=1.2566370614359172, dist=28),
    #     Ray(origin=(15, 19), angle=0.9424777960769379, dist=28),
    #     Ray(origin=(15, 20), angle=0.6283185307179586, dist=27),
    #     Ray(origin=(15, 21), angle=0.3141592653589793, dist=26),
    #     Ray(origin=(15, 22), angle=0.0, dist=29),
    #     Ray(origin=(15, 23), angle=0.3141592653589793, dist=50),
    #     Ray(origin=(15, 24), angle=0.6283185307179586, dist=44),
    #     Ray(origin=(15, 26), angle=0.9424777960769379, dist=20),
    #     Ray(origin=(15, 28), angle=1.2566370614359172, dist=19),
    #     Ray(origin=(15, 29), angle=1.5707963267948966, dist=20),
    #     Ray(origin=(15, 30), angle=1.8849555921538759, dist=29),
    #     Ray(origin=(15, 30), angle=2.199114857512855, dist=31),
    #     Ray(origin=(15, 32), angle=2.5132741228718345, dist=30),
    #     Ray(origin=(15, 33), angle=2.827433388230814, dist=32),
    #     Ray(origin=(15, 34), angle=3.141592653589793, dist=29),
    #     Ray(origin=(15, 35), angle=2.827433388230814, dist=64),
    #     Ray(origin=(15, 37), angle=2.5132741228718345, dist=33),
    #     Ray(origin=(15, 40), angle=2.199114857512855, dist=22),
    #     Ray(origin=(15, 41), angle=1.8849555921538759, dist=22),
    #     Ray(origin=(15, 42), angle=1.5707963267948966, dist=27),
    #     Ray(origin=(15, 42), angle=1.2566370614359172, dist=28),
    #     Ray(origin=(15, 43), angle=0.9424777960769379, dist=18),
    #     Ray(origin=(15, 44), angle=0.6283185307179586, dist=17),
    #     Ray(origin=(15, 45), angle=0.3141592653589793, dist=16),
    #     Ray(origin=(15, 45), angle=0.0, dist=25),
    #     Ray(origin=(15, 46), angle=0.3141592653589793, dist=50),
    #     Ray(origin=(15, 47), angle=0.6283185307179586, dist=15),
    #     Ray(origin=(15, 48), angle=0.9424777960769379, dist=13),
    #     Ray(origin=(15, 49), angle=1.2566370614359172, dist=28),
    #     Ray(origin=(15, 50), angle=1.5707963267948966, dist=32),
    #     Ray(origin=(15, 51), angle=1.8849555921538759, dist=29),
    #     Ray(origin=(15, 51), angle=2.199114857512855, dist=28),
    #     Ray(origin=(15, 53), angle=2.5132741228718345, dist=14),
    #     Ray(origin=(15, 54), angle=2.827433388230814, dist=12),
    #     Ray(origin=(15, 54), angle=3.141592653589793, dist=12),
    #     Ray(origin=(15, 55), angle=2.827433388230814, dist=12),
    #     Ray(origin=(15, 55), angle=2.5132741228718345, dist=12),
    #     Ray(origin=(15, 55), angle=2.199114857512855, dist=12),
    #     Ray(origin=(15, 56), angle=1.8849555921538759, dist=26),
    #     Ray(origin=(15, 57), angle=1.5707963267948966, dist=25),
    #     Ray(origin=(15, 57), angle=1.2566370614359172, dist=23),
    #     Ray(origin=(15, 58), angle=0.9424777960769379, dist=23),
    #     Ray(origin=(15, 59), angle=0.6283185307179586, dist=25),
    #     Ray(origin=(15, 60), angle=0.3141592653589793, dist=29)]

    # rays = [Ray(origin=(100, 10), angle=1.2566370614359172, dist=29), Ray(origin=(100, 10), angle=0.9424777960769379, dist=28), Ray(origin=(100, 10), angle=0.6283185307179586, dist=30), Ray(origin=(100, 10), angle=0.3141592653589793, dist=65), Ray(origin=(100, 10), angle=0.0, dist=50), Ray(origin=(100, 10), angle=0.3141592653589793, dist=50), Ray(origin=(100, 10), angle=0.6283185307179586, dist=50), Ray(origin=(100, 10), angle=0.9424777960769379, dist=50), Ray(origin=(100, 10), angle=1.2566370614359172, dist=32), Ray(origin=(100, 10), angle=1.5707963267948966, dist=33), Ray(origin=(100, 10), angle=1.8849555921538759, dist=50), Ray(origin=(100, 10), angle=2.199114857512855, dist=34), Ray(origin=(100, 10), angle=2.5132741228718345, dist=28), Ray(origin=(100, 10), angle=2.827433388230814, dist=28), Ray(origin=(100, 10), angle=3.141592653589793, dist=30), Ray(origin=(100, 10), angle=2.827433388230814, dist=50), Ray(origin=(100, 12), angle=2.5132741228718345, dist=29), Ray(origin=(100, 13), angle=2.199114857512855, dist=26), Ray(origin=(100, 15), angle=1.8849555921538759, dist=25), Ray(origin=(100, 16), angle=1.5707963267948966, dist=40), Ray(origin=(100, 17), angle=1.2566370614359172, dist=42), Ray(origin=(100, 18), angle=0.9424777960769379, dist=28), Ray(origin=(100, 19), angle=0.6283185307179586, dist=21), Ray(origin=(100, 20), angle=0.3141592653589793, dist=22), Ray(origin=(100, 21), angle=0.0, dist=49), Ray(origin=(100, 22), angle=0.3141592653589793, dist=50), Ray(origin=(100, 24), angle=0.6283185307179586, dist=20), Ray(origin=(100, 25), angle=0.9424777960769379, dist=18), Ray(origin=(100, 26), angle=1.2566370614359172, dist=30), Ray(origin=(
    #     100, 27), angle=1.5707963267948966, dist=29), Ray(origin=(100, 28), angle=1.8849555921538759, dist=31), Ray(origin=(100, 28), angle=2.199114857512855, dist=29), Ray(origin=(100, 30), angle=2.5132741228718345, dist=32), Ray(origin=(100, 31), angle=2.827433388230814, dist=27), Ray(origin=(100, 32), angle=3.141592653589793, dist=31), Ray(origin=(100, 33), angle=2.827433388230814, dist=50), Ray(origin=(100, 34), angle=2.5132741228718345, dist=51), Ray(origin=(100, 37), angle=2.199114857512855, dist=19), Ray(origin=(100, 39), angle=1.8849555921538759, dist=21), Ray(origin=(100, 40), angle=1.5707963267948966, dist=25), Ray(origin=(100, 40), angle=1.2566370614359172, dist=35), Ray(origin=(100, 42), angle=0.9424777960769379, dist=37), Ray(origin=(100, 43), angle=0.6283185307179586, dist=55), Ray(origin=(100, 46), angle=0.3141592653589793, dist=12), Ray(origin=(100, 47), angle=0.0, dist=10), Ray(origin=(100, 48), angle=0.3141592653589793, dist=11), Ray(origin=(100, 48), angle=0.6283185307179586, dist=10), Ray(origin=(100, 48), angle=0.9424777960769379, dist=15), Ray(origin=(100, 49), angle=1.2566370614359172, dist=32), Ray(origin=(100, 50), angle=1.5707963267948966, dist=27), Ray(origin=(100, 51), angle=1.8849555921538759, dist=25), Ray(origin=(100, 51), angle=2.199114857512855, dist=25), Ray(origin=(100, 52), angle=2.5132741228718345, dist=44), Ray(origin=(100, 54), angle=2.827433388230814, dist=32), Ray(origin=(100, 56), angle=3.141592653589793, dist=18), Ray(origin=(100, 57), angle=2.827433388230814, dist=17), Ray(origin=(100, 58), angle=2.5132741228718345, dist=17), Ray(origin=(100, 58), angle=2.199114857512855, dist=29), Ray(origin=(100, 59), angle=1.8849555921538759, dist=30)]

    # mapper = Mapper(100, dist_cutoff=25, connect_cutoff=25)
    # for i, ray in enumerate(rays):
    #     ray = Ray(
    #         (ray.origin[0] - 50, 10 + (ray.origin[1] - 10) * 2),
    #         ray.angle,
    #         ray.dist / 2
    #     )
    #     mapper.add_ray(ray)

    #     # if i % 5 == 0:
    #     mapper.plot()

    # mapper = Mapper(60, 20, 20)
    # with open("./map-1676358884.1629941.npy", "rb") as f:
    #     mapper.data = np.load(f)[20:80, 45:105]
    # mapper.plot(None)

    # path = mapper.route((35, 0), (35, 40))
    # print(path)
    # mapper.plot(path)

    # mapper.data = mapper.data == Mapper.FILLED
    # mapper.plot(path)
    pass
