# File: /raspberry-pi-robot/raspberry-pi-robot/src/navigation/path_planning.py

"""
This module implements path planning algorithms for the robot, allowing it to navigate to target coordinates while avoiding obstacles.
The algorithms included can be A*, RRT, or D* Lite. The choice of algorithm can be adjusted based on the specific requirements of the navigation task.
"""

import numpy as np
import heapq

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to this node
        self.h = 0  # Heuristic cost to goal
        self.f = 0  # Total cost

    def __eq__(self, other):
        return self.position == other.position

def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def a_star(start, goal, grid):
    open_list = []
    closed_list = set()

    start_node = Node(start)
    goal_node = Node(goal)

    heapq.heappush(open_list, (start_node.f, start_node))

    while open_list:
        current_node = heapq.heappop(open_list)[1]
        closed_list.add(current_node.position)

        if current_node == goal_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]  # Return reversed path

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4 possible movements (up, right, down, left)

        for new_position in neighbors:
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if (0 <= node_position[0] < grid.shape[0]) and (0 <= node_position[1] < grid.shape[1]):
                if grid[node_position[0]][node_position[1]] != 0:  # Check if the cell is not an obstacle
                    continue

                neighbor_node = Node(node_position, current_node)

                if neighbor_node.position in closed_list:
                    continue

                neighbor_node.g = current_node.g + 1
                neighbor_node.h = heuristic(neighbor_node.position, goal_node.position)
                neighbor_node.f = neighbor_node.g + neighbor_node.h

                if not any(neighbor_node == open_node[1] and neighbor_node.g > open_node[1].g for open_node in open_list):
                    heapq.heappush(open_list, (neighbor_node.f, neighbor_node))

    return None  # Return None if no path is found

def move_to_coordinate(start, goal, grid):
    path = a_star(start, goal, grid)
    if path:
        print("Path found:", path)
        return path
    else:
        print("No path found.")
        return None

# Example usage (to be replaced with actual robot grid and coordinates):
if __name__ == "__main__":
    grid = np.zeros((10, 10))  # Example grid (0 = free space, 1 = obstacle)
    grid[5][5] = 1  # Adding an obstacle
    start = (0, 0)
    goal = (7, 7)
    move_to_coordinate(start, goal, grid)