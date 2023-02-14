import heapq
import numpy as np


def astar(array, begin, dest):
    """
    set array coming from map.data,whihc is two 2d array.
    """
    print(array.sum(), begin, dest)
    print(array)
    begin = tuple(begin)
    dest = tuple(dest)

    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    #  (1, 1), (1, -1), (-1, 1), (-1, -1)]

    openlist = []
    gc = {begin: 0}
    close_set = set()
    came_from = {}
    totalcost = {begin: heur(begin, dest)}
    heapq.heappush(openlist, (totalcost[begin], begin))

    while openlist:
        current = heapq.heappop(openlist)[1]
        if current == dest:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(begin)
            path.reverse()
            return path
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_gc = gc[current] + heur(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_gc >= gc.get(neighbor, 0):
                continue
            if tentative_gc < gc.get(neighbor, 0) or neighbor not in [i[1] for i in openlist]:
                came_from[neighbor] = current
                gc[neighbor] = tentative_gc
                totalcost[neighbor] = tentative_gc + heur(neighbor, dest)
                heapq.heappush(openlist, (totalcost[neighbor], neighbor))
    return None


def heur(a, b):
    return (((a[0]-b[0])**2+(a[1] - b[1])**2)) ** 0.5


# array = np.array([[0, 0, 0, 0],
#                  [0, 0, 0, 0],
#                  [0, 0, 0, 0],
#                  [0, 0, 0, 0],
#                   [0, 0, 0, 0]])
#
# array = np.array([[0, 0, 0, 0],
#          [0, 1, 1, 0],
#          [0, 1, 0, 0],
#          [0, 0, 0, 0]])


array = np.array([[0, 0, 0, 0],
                  [1, 1, 1, 0],
                  [0, 0, 0, 0],
                  [0, 0, 0, 0]])

start = (0, 0)
end = (3, 1)

path = astar(array, start, end)
print(path)
