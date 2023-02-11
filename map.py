import numpy as np
from typing import NamedTuple, Iterable

Ray = NamedTuple(
        "Ray", 
        [
            ('origin', Iterable), 
            ('angle', float), 
            ('dist', int), 
        ])


class LocationTracker:
    """Keeps track of current location of the car"""
    pass


class Mapper:
    """Maps the environent"""

    EMPTY = 0
    UNKNOWN = 1
    FILLED = 2

    def __init__(self, size=100):
        self.size = size
        self.data = np.ones((size, size)) * Mapper.UNKNOWN
        self.rays = []

    def add_ray(self, ray: Ray):
        x, y = ray.origin
        self.data[x, y] = 0
        # Todo: calculate area
        self.rays.append(ray)

    def __str__(self):
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

    def plot(self, show=True, save_file=None):
        import matplotlib.pyplot as plt
        
        plt.pcolormesh(self.data, cmap='Greys', edgecolors='k', linewidth=1)
        for ray in self.rays:
            x, y = ray.origin
            dx = np.cos(ray.angle) * ray.dist
            dy = np.sin(ray.angle) * ray.dist
            print(x, y, dx, dy)
            plt.arrow(x, y, dx, dy, fc="k", ec="k", color='red',
                      head_width=0.06, head_length=0.1)

        plt.gca().invert_yaxis() # origin from bottom left
        plt.gca().set_aspect('equal')

        if save_file and isinstance(save_file, str):
            plt.savefig(save_file, dpi=300)
        if show:
            plt.show()


if __name__ == '__main__':
    mapper = Mapper(50)
    mapper.add_ray(Ray((25, 25), 0, 10))
    mapper.add_ray(Ray((25, 25), 20 / 180 * np.pi, 10))
    mapper.plot(show=False, save_file="map0.png")
    print(mapper)
