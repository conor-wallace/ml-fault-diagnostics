import matplotlib.pyplot as plt
import numpy as np
import math
import sys
import rospy
from network_faults.msg import Coordinate, Path

class Node():
    def __init__(self, parent, position):
        self.previous = parent
        self.position = position
        self.neighbors = []
        self.f = 0
        self.g = 0
        self.h = 0
        self.wall = 0

    def addNeighbors(self, grid):
        x = self.position[0]
        y = self.position[1]
        if x < cols - 1:
            self.neighbors.append(grid[x + 1, y])
        if y < rows - 1:
            self.neighbors.append(grid[x, y + 1])
        if y > 0:
            self.neighbors.append(grid[x, y - 1])
        if (x < cols - 1) and (y < rows - 1):
            self.neighbors.append(grid[x + 1, y + 1])

cols = 40
rows = 40
w = 0
h = 0
start = None
end = None
grid = np.empty((rows, cols), dtype=object)
open_list = []
closed_list = []
path = []

def removeFromArray(array, elem):
    i = len(array) - 1
    while i >= 0:
        if array[i] == elem:
            del array[i]
            break
        i -= 1

def heuristic(a, b):
    d = math.sqrt(((b.position[0] - a.position[0]) ** 2) + ((b.position[1] - a.position[1]) ** 2))
    return d

def setup():
    global grid, open_list, closed_list, w, h, start, end

    w = 640 / cols
    h = 640 / rows

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            grid[i, j] = Node(None, [i, j])

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            grid[i, j].addNeighbors(grid)

    wall1 = [[20,30],[21,30],[22,30],[23,30],[24,30],[25,30],[26,30],[27,30],[28,30],[29,30],[30,30]]
    wall1 = np.reshape(wall1, (11, 2))
    wall2 = np.flip(wall1, 1)

    wall1_count = 0
    wall2_count = 0

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if wall1_count <= 10:
                if i == wall1[wall1_count, 0] and j == wall1[wall1_count, 1]:
                    grid[i, j].wall = 1
                    wall1_count += 1
            if wall2_count <= 10:
                if i == wall2[wall2_count, 0] and j == wall2[wall2_count, 1]:
                    grid[i, j].wall = 1
                    wall2_count += 1

    start = grid[0, 0]
    end = grid[35, 35]
    open_list.append(start)

def astar():
    global grid, open_list, closed_list, start, end, path
    #A* algorithm
    while len(open_list) > 0:
        winner = 0
        for i in range(len(open_list)):
            if open_list[i].f < open_list[winner].f:
                winner = i

        current = open_list[winner]
        if current == end:
            print("done")
            temp = current
            path = []
            path.append(temp.position)
            while temp.previous:
                path.append(temp.previous.position)
                temp = temp.previous
            return path

        removeFromArray(open_list, current)
        closed_list.append(current)

        neighbors = current.neighbors
        for i in range(len(neighbors)):
            neighbor = neighbors[i]
            temp_g = 0
            if not neighbor in closed_list and neighbor.wall != 1:
                temp_g = current.g + 1

                new_path = 0
                if neighbor in open_list:
                    if temp_g < neighbor.g:
                        neighbor.g = temp_g
                        new_path = 1
                else:
                    neighbor.g = temp_g
                    new_path = 1
                    open_list.append(neighbor)

                if new_path == 1:
                    neighbor.h = heuristic(neighbor, end)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.previous = current

setup()
path = astar()
path.append([-1000, -1000])

# print(path)
# fig, (ax1) = plt.subplots(1, 1)
#
# ax1.plot(path[:, 0], path[:, 1], color='b')
# ax1.set(xlabel='x', ylabel='y', title='Optimal Path')
#
# plt.show()

sent = 0
rospy.init_node('path_planner')
path_pub = rospy.Publisher('/path', Path, queue_size=1, tcp_nodelay=True)
pathPub = Path()
points = []
for i in range(len(path)):
    print(path[i])
    coordinate = Coordinate()
    coordinate.coordinate = path[i]
    points.append(coordinate)

pathPub.path = points
rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    path_pub.publish(pathPub)
    rate.sleep()
