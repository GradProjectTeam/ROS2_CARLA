import numpy as np
from queue import PriorityQueue

class Node:
    def __init__(self, x, y, theta, cost, parent=None):
        self.x = x
        self.y = y
        self.theta = theta  # orientation in radians
        self.cost = cost
        self.parent = parent

class HybridAStar:
    def __init__(self, cost_map, grid_size=1.0, wheelbase=2.5):
        self.cost_map = cost_map
        self.rows, self.cols = cost_map.shape
        self.grid_size = grid_size
        self.wheelbase = wheelbase
        self.obstacle_thresh = 50  # Cells >= this value are obstacles

    def heuristic(self, a, b):
        return np.hypot(a.x - b.x, a.y - b.y)

    def is_in_bounds(self, x, y):
        return 0 <= x < self.cols and 0 <= y < self.rows

    def is_obstacle(self, x, y):
        xi, yi = int(round(x)), int(round(y))
        return not self.is_in_bounds(xi, yi) or self.cost_map[yi][xi] >= self.obstacle_thresh

    def kinematic_motion(self, node, steering, velocity, dt=1.0):
        x = node.x + velocity * np.cos(node.theta) * dt
        y = node.y + velocity * np.sin(node.theta) * dt
        theta = node.theta + (velocity / self.wheelbase) * np.tan(steering) * dt
        return Node(x, y, theta, node.cost + dt, parent=node)

    def get_successors(self, node):
        motions = []
        for delta in np.linspace(-0.5, 0.5, 5):  # Steering angles
            next_node = self.kinematic_motion(node, delta, 1.0)
            if not self.is_obstacle(next_node.x, next_node.y):
                motions.append(next_node)
        return motions

    def plan(self, start, goal):
        open_set = PriorityQueue()
        closed_set = set()

        open_set.put((0, start))
        visited = set()

        while not open_set.empty():
            _, current = open_set.get()
            key = (round(current.x, 1), round(current.y, 1), round(current.theta, 2))

            if key in visited:
                continue
            visited.add(key)

            if np.hypot(current.x - goal.x, current.y - goal.y) < 1.0:
                path = []
                while current:
                    path.append((current.x, current.y, current.theta))
                    current = current.parent
                return path[::-1]

            for neighbor in self.get_successors(current):
                h = self.heuristic(neighbor, goal)
                open_set.put((neighbor.cost + h, neighbor))

        return None
