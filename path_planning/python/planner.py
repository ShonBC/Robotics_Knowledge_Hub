import heapq
import numpy as np
import random
from typing import List, Tuple, Dict
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon, Arc
from matplotlib.collections import LineCollection

class Planner:
    """
    A class for implementing various path planning algorithms.

    This class provides methods for different path planning algorithms including A*, Weighted A*,
    Dijkstra's algorithm, RRT (Rapidly-exploring Random Tree), and RRT* (Optimal RRT).
    It also includes visualization capabilities for the path planning process.

    Attributes:
        graph (Dict): A dictionary representing the graph for graph-based algorithms.
        obstacles (List[Tuple[float, float, str, List[float]]]): A list of obstacle definitions.
        start (Tuple[int, int]): The start point coordinates.
        goal (Tuple[int, int]): The goal point coordinates.
        fig (matplotlib.figure.Figure): The matplotlib figure for visualization.
        ax (matplotlib.axes.Axes): The matplotlib axes for plotting.
        map_size (int): The size of the square map (default is 100x100).
        move_set (List[Tuple[int, int]]): The set of possible moves for exploration.
        step_size (int): The step size for moves.
        robot_buffer (float): The buffer zone around the robot to account for its collision area.
        goal_tolerance (float): The tolerance zone around the goal.

    Methods:
        set_graph(graph: Dict): Sets the graph for graph-based algorithms.
        set_obstacles(obstacles: List[Tuple[float, float, str, List[float]]]): Sets the obstacles for the map.
        set_start_goal(start: Tuple[int, int], goal: Tuple[int, int]): Sets the start and goal points.
        set_move_set(move_set: List[Tuple[int, int]], step_size: int): Sets the move set and step size.
        set_robot_buffer(buffer: float): Sets the buffer zone around the robot.
        set_goal_tolerance(tolerance: float): Sets the tolerance zone around the goal.
        heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float: Calculates the heuristic distance between two points.
        initialize_plot(): Initializes the plot for visualization.
        update_plot(current, neighbors=None): Updates the plot during the path planning process.
        plot_final_path(path): Plots the final path found by the algorithm.
        plot_rounded_end(left_point, right_point, center_point, start=True): Plots a circle at the end of the path.
        create_offset_paths(path, offset): Creates offset outlines of the path on both sides.
        a_star(visualize: bool = False) -> List[Tuple[int, int]]: Implements the A* algorithm.
        weighted_a_star(weight: float = 1.5, visualize: bool = False) -> List[Tuple[int, int]]: Implements the Weighted A* algorithm.
        dijkstra(visualize: bool = False) -> List[Tuple[int, int]]: Implements Dijkstra's algorithm.
        rrt(max_iterations: int = 1000, visualize: bool = False) -> List[Tuple[int, int]]: Implements the RRT algorithm.
        rrt_star(max_iterations: int = 1000, search_radius: float = 5, visualize: bool = False) -> List[Tuple[int, int]]: Implements the RRT* algorithm.
        slam(visualize: bool = False): Placeholder for SLAM implementation.
        random_state() -> Tuple[int, int]: Generates a random state within the planning space.
        steer(from_point: Tuple[int, int], to_point: Tuple[int, int]) -> Tuple[int, int]: Steers from one point towards another with the defined step size.
        collision_free(from_point: Tuple[int, int], to_point: Tuple[int, int]) -> bool: Checks if a path between two points is collision-free.
        extract_path(tree: Dict, end_node: Tuple[int, int]) -> List[Tuple[int, int]]: Extracts the path from the tree structure used in RRT and RRT*.
        is_goal_reached(point: Tuple[int, int]) -> bool: Checks if a point is within the goal tolerance zone.
        point_in_collision(point: Tuple[int, int]) -> bool: Checks if a given point collides with any obstacle in the environment.
    """

    def __init__(self):
        self.graph = {}  # For graph-based algorithms
        self.obstacles = []  # For sampling-based algorithms
        self.start = None
        self.goal = None
        self.fig = None
        self.ax = None
        self.map_size = 100  # 100x100 map
        self.move_set = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Default move set
        self.step_size = 1  # Default step size
        self.robot_buffer = 2  # Default robot buffer
        self.goal_tolerance = 5  # Default goal tolerance

    def set_graph(self, graph: Dict):
        """
        Sets the graph for graph-based algorithms.

        Args:
            graph (Dict): A dictionary representing the graph where keys are nodes and values are lists of neighboring nodes.
        """
        self.graph = graph

    def set_obstacles(self, obstacles: List[Tuple[float, float, str, List[float]]]):
        """
        Sets the obstacles for the map.

        Args:
            obstacles (List[Tuple[float, float, str, List[float]]]): A list of obstacle definitions.
                Each obstacle is defined by (x, y, shape, params) where:
                - x, y are the center coordinates
                - shape is a string ('rectangle', 'circle', or 'polygon')
                - params are the shape parameters (width and height for rectangle, radius for circle, points for polygon)
        """
        self.obstacles = obstacles

    def set_start_goal(self, start: Tuple[int, int], goal: Tuple[int, int]):
        """
        Sets the start and goal points for the path planning problem.

        Args:
            start (Tuple[int, int]): The start point coordinates.
            goal (Tuple[int, int]): The goal point coordinates.
        """
        self.start = start
        self.goal = goal

    def set_move_set(self, move_set: List[Tuple[int, int]], step_size: int):
        """
        Sets the move set and step size for map exploration.

        Args:
            move_set (List[Tuple[int, int]]): A list of possible moves as (dx, dy) tuples.
            step_size (int): The step size for moves.
        """
        self.move_set = move_set
        self.step_size = step_size

    def set_robot_buffer(self, buffer: float):
        """
        Sets the buffer zone around the robot to account for its collision area.

        Args:
            buffer (float): The radius of the buffer zone around the robot.
        """
        self.robot_buffer = buffer

    def set_goal_tolerance(self, tolerance: float):
        """
        Sets the tolerance zone around the goal.

        Args:
            tolerance (float): The radius of the tolerance zone around the goal.
        """
        self.goal_tolerance = tolerance

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """
        Calculates the heuristic distance between two points.

        Args:
            a (Tuple[int, int]): The first point.
            b (Tuple[int, int]): The second point.

        Returns:
            float: The Euclidean distance between the two points.
        """
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def initialize_plot(self):
        """
        Initializes the plot for visualization.

        This method sets up the matplotlib figure and axes, and plots the obstacles, start, and goal points.
        """
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(0, self.map_size)
        self.ax.set_ylim(0, self.map_size)
        
        for obstacle in self.obstacles:
            x, y, shape, params = obstacle
            if shape == 'rectangle':
                width, height = params
                self.ax.add_patch(Rectangle((x-width/2, y-height/2), width, height, fill=True, color='gray'))
            elif shape == 'circle':
                radius = params[0]
                self.ax.add_patch(Circle((x, y), radius, fill=True, color='gray'))
            elif shape == 'polygon':
                self.ax.add_patch(Polygon(params, fill=True, color='gray'))
        
        self.ax.plot(*self.start, 'go', markersize=10)
        self.ax.plot(*self.goal, 'ro', markersize=10)
        self.ax.add_patch(Circle(self.goal, self.goal_tolerance, fill=False, color='r', linestyle='--'))

    def update_plot(self, current, neighbors=None):
        """
        Updates the plot during the path planning process.

        Args:
            current (Tuple[int, int]): The current node being explored.
            neighbors (List[Tuple[int, int]], optional): The neighbors of the current node. Defaults to None.
        """
        if neighbors:
            for neighbor in neighbors:
                if self.collision_free(current, neighbor):
                    self.ax.plot([current[0], neighbor[0]], [current[1], neighbor[1]], 'y-', linewidth=0.5)
        self.ax.plot(*current, 'bo', markersize=3)
        plt.pause(0.001)

    def plot_final_path(self, path):
        """
        Plots the final path found by the algorithm.

        Args:
            path (List[Tuple[int, int]]): The final path as a list of coordinate tuples.
        """
        path_x, path_y = zip(*path)
        self.ax.plot(path_x, path_y, 'g-', linewidth=2)
        
        # Create offset paths on both sides
        left_offset_path, right_offset_path = self.create_offset_paths(path, self.robot_buffer)
        
        # Plot offset paths
        left_lines = LineCollection(left_offset_path, colors='g', linestyles='--', linewidths=1)
        right_lines = LineCollection(right_offset_path, colors='g', linestyles='--', linewidths=1)
        self.ax.add_collection(left_lines)
        self.ax.add_collection(right_lines)
        
        # Connect the ends with rounded corners
        self.plot_rounded_end(left_offset_path[0][0], right_offset_path[0][0], path[0], start=True)
        self.plot_rounded_end(left_offset_path[-1][1], right_offset_path[-1][1], path[-1], start=False)
        
        plt.pause(0.001)

    def plot_rounded_end(self, left_point, right_point, center_point, start=True):
        """
        Plots a circle at the end of the path indicating the robot's buffer size.

        Args:
            left_point (Tuple[float, float]): The left point of the offset path.
            right_point (Tuple[float, float]): The right point of the offset path.
            center_point (Tuple[int, int]): The center point (start or end of the original path).
            start (bool): True if this is the start of the path, False if it's the end.
        """
        # Draw a circle at the center point with radius equal to the robot's buffer
        circle = Circle(center_point, self.robot_buffer, fill=False, color='g', linestyle='--', linewidth=1)
        self.ax.add_patch(circle)

    def create_offset_paths(self, path, offset):
        """
        Creates offset outlines of the path on both sides.

        Args:
            path (List[Tuple[int, int]]): The original path.
            offset (float): The offset distance.

        Returns:
            Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]]]: The left and right offset path segments.
        """
        left_offset_path = []
        right_offset_path = []
        for i in range(len(path) - 1):
            start = path[i]
            end = path[i+1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = np.sqrt(dx**2 + dy**2)
            ux = -dy / length
            uy = dx / length
            left_offset_start = (start[0] + offset * ux, start[1] + offset * uy)
            left_offset_end = (end[0] + offset * ux, end[1] + offset * uy)
            right_offset_start = (start[0] - offset * ux, start[1] - offset * uy)
            right_offset_end = (end[0] - offset * ux, end[1] - offset * uy)
            left_offset_path.append([left_offset_start, left_offset_end])
            right_offset_path.append([right_offset_start, right_offset_end])
        return left_offset_path, right_offset_path

    def a_star(self, visualize: bool = False) -> List[Tuple[int, int]]:
        """
        Performs A* search algorithm to find the optimal path from start to goal.

        Args:
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[int, int]]: The optimal path from start to goal as a list of coordinate tuples.
                                   Returns an empty list if no path is found.
        """
        if visualize:
            self.initialize_plot()

        open_set = [(0, self.start)]
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self.heuristic(self.start, self.goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if visualize:
                self.update_plot(current)

            if self.is_goal_reached(current):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for dx, dy in self.move_set:
                neighbor = (current[0] + dx * self.step_size, current[1] + dy * self.step_size)
                if 0 <= neighbor[0] < self.map_size and 0 <= neighbor[1] < self.map_size:
                    if self.collision_free(current, neighbor):
                        tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                        if tentative_g_score < g_score.get(neighbor, float('inf')):
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, self.goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def weighted_a_star(self, weight: float = 1.5, visualize: bool = False) -> List[Tuple[int, int]]:
        """
        Performs Weighted A* search algorithm to find a path from start to goal.

        Args:
            weight (float): The weight to apply to the heuristic. Defaults to 1.5.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[int, int]]: The path from start to goal as a list of coordinate tuples.
                                   Returns an empty list if no path is found.
        """
        if visualize:
            self.initialize_plot()

        open_set = [(0, self.start)]
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: weight * self.heuristic(self.start, self.goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if visualize:
                self.update_plot(current)

            if self.is_goal_reached(current):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for dx, dy in self.move_set:
                neighbor = (current[0] + dx * self.step_size, current[1] + dy * self.step_size)
                if 0 <= neighbor[0] < self.map_size and 0 <= neighbor[1] < self.map_size:
                    if self.collision_free(current, neighbor):
                        tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                        if tentative_g_score < g_score.get(neighbor, float('inf')):
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = g_score[neighbor] + weight * self.heuristic(neighbor, self.goal)
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def dijkstra(self, visualize: bool = False) -> List[Tuple[int, int]]:
        """
        Performs Dijkstra's algorithm to find the shortest path from start to goal.

        Args:
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[int, int]]: The shortest path from start to goal as a list of coordinate tuples.
                                   Returns an empty list if no path is found.
        """
        if visualize:
            self.initialize_plot()

        open_set = [(0, self.start)]
        came_from = {}
        g_score = {self.start: 0}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if visualize:
                self.update_plot(current)

            if self.is_goal_reached(current):
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for dx, dy in self.move_set:
                neighbor = (current[0] + dx * self.step_size, current[1] + dy * self.step_size)
                if 0 <= neighbor[0] < self.map_size and 0 <= neighbor[1] < self.map_size:
                    if self.collision_free(current, neighbor):
                        tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                        if tentative_g_score < g_score.get(neighbor, float('inf')):
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            heapq.heappush(open_set, (g_score[neighbor], neighbor))

        return []

    def rrt(self, max_iterations: int = 1000, visualize: bool = False) -> List[Tuple[int, int]]:
        """
        Performs Rapidly-exploring Random Tree (RRT) algorithm to find a path from start to goal.

        Args:
            max_iterations (int): The maximum number of iterations to perform. Defaults to 1000.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[int, int]]: The path from start to goal as a list of coordinate tuples.
                                   Returns an empty list if no path is found.
        """
        if visualize:
            self.initialize_plot()

        tree = {self.start: None}
        
        for _ in range(max_iterations):
            random_point = self.random_state()
            nearest_node = min(tree, key=lambda n: self.heuristic(n, random_point))
            
            new_node = self.steer(nearest_node, random_point)
            if self.collision_free(nearest_node, new_node):
                tree[new_node] = nearest_node
                
                if visualize:
                    self.update_plot(new_node)
                    self.ax.plot([nearest_node[0], new_node[0]], [nearest_node[1], new_node[1]], '-', color='violet')

                if self.is_goal_reached(new_node):
                    path = self.extract_path(tree, new_node)
                    if visualize:
                        self.plot_final_path(path)
                    return path
        
        return []

    def rrt_star(self, max_iterations: int = 1000, search_radius: float = 5, visualize: bool = False) -> List[Tuple[int, int]]:
        """
        Performs RRT* (Optimal Rapidly-exploring Random Tree) algorithm to find an optimal path from start to goal.

        Args:
            max_iterations (int): The maximum number of iterations to perform. Defaults to 1000.
            search_radius (float): The radius within which nodes are considered for rewiring. Defaults to 5.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[int, int]]: The optimal path from start to goal as a list of coordinate tuples.
                                   Returns an empty list if no path is found.
        """
        if visualize:
            self.initialize_plot()

        tree = {self.start: None}
        costs = {self.start: 0}
        
        for _ in range(max_iterations):
            random_point = self.random_state()
            nearest_node = min(tree, key=lambda n: self.heuristic(n, random_point))
            
            new_node = self.steer(nearest_node, random_point)
            if self.collision_free(nearest_node, new_node):
                near_nodes = [n for n in tree if self.heuristic(n, new_node) < search_radius]
                tree[new_node] = nearest_node
                costs[new_node] = costs[nearest_node] + self.heuristic(nearest_node, new_node)
                
                if visualize:
                    self.update_plot(new_node)
                    self.ax.plot([nearest_node[0], new_node[0]], [nearest_node[1], new_node[1]], '-', color='violet')

                for near_node in near_nodes:
                    if self.collision_free(near_node, new_node):
                        cost = costs[near_node] + self.heuristic(near_node, new_node)
                        if cost < costs[new_node]:
                            tree[new_node] = near_node
                            costs[new_node] = cost
                
                for near_node in near_nodes:
                    if near_node != tree[new_node]:
                        cost = costs[new_node] + self.heuristic(new_node, near_node)
                        if cost < costs[near_node] and self.collision_free(new_node, near_node):
                            old_parent = tree[near_node]
                            tree[near_node] = new_node
                            costs[near_node] = cost
                            
                            if visualize:
                                self.ax.plot([old_parent[0], near_node[0]], [old_parent[1], near_node[1]], 'w-')
                                self.ax.plot([new_node[0], near_node[0]], [new_node[1], near_node[1]], '-', color='lightblue')
                
                if self.is_goal_reached(new_node):
                    path = self.extract_path(tree, new_node)
                    if visualize:
                        self.plot_final_path(path)
                    return path
        
        return []

    def slam(self, visualize: bool = False):
        """
        Placeholder for Simultaneous Localization and Mapping (SLAM) implementation.

        Args:
            visualize (bool): If True, would visualize the SLAM process. Defaults to False.

        Returns:
            None
        """
        print("SLAM algorithm not implemented. It requires sensor data and more complex processing.")
        if visualize:
            print("Visualization for SLAM would typically show the robot's estimated position and the map being built.")
        return None

    def random_state(self) -> Tuple[int, int]:
        """
        Generate a random state within the planning space.

        Returns:
            Tuple[int, int]: A tuple representing a random (x, y) coordinate within the map.
        """
        return (random.randint(0, self.map_size-1), random.randint(0, self.map_size-1))

    def steer(self, from_point: Tuple[int, int], to_point: Tuple[int, int]) -> Tuple[int, int]:
        """
        Steer from one point towards another with the defined step size.

        Args:
            from_point (Tuple[int, int]): The starting point.
            to_point (Tuple[int, int]): The target point to steer towards.

        Returns:
            Tuple[int, int]: The new point after steering.
        """
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        if distance < self.step_size:
            return to_point
        else:
            theta = np.arctan2(dy, dx)
            return (int(from_point[0] + self.step_size * np.cos(theta)),
                    int(from_point[1] + self.step_size * np.sin(theta)))

    def collision_free(self, from_point: Tuple[int, int], to_point: Tuple[int, int]) -> bool:
        """
        Check if a path between two points is collision-free, considering the robot's buffer zone and map edges.

        Args:
            from_point (Tuple[int, int]): The starting point of the path.
            to_point (Tuple[int, int]): The ending point of the path.

        Returns:
            bool: True if the path is collision-free, False otherwise.
        """
        # Check collision with map edges
        if (from_point[0] < self.robot_buffer or from_point[0] >= self.map_size - self.robot_buffer or
            from_point[1] < self.robot_buffer or from_point[1] >= self.map_size - self.robot_buffer or
            to_point[0] < self.robot_buffer or to_point[0] >= self.map_size - self.robot_buffer or
            to_point[1] < self.robot_buffer or to_point[1] >= self.map_size - self.robot_buffer):
            return False

        # Check collision with obstacles
        for obstacle in self.obstacles:
            x, y, shape, params = obstacle
            if shape == 'rectangle':
                width, height = params
                rect = Rectangle((x-width/2, y-height/2), width, height)
                if (rect.contains_point((from_point[0], from_point[1])) or 
                    rect.contains_point((to_point[0], to_point[1])) or
                    rect.contains_point((from_point[0] + self.robot_buffer, from_point[1])) or
                    rect.contains_point((from_point[0] - self.robot_buffer, from_point[1])) or
                    rect.contains_point((from_point[0], from_point[1] + self.robot_buffer)) or
                    rect.contains_point((from_point[0], from_point[1] - self.robot_buffer)) or
                    rect.contains_point((to_point[0] + self.robot_buffer, to_point[1])) or
                    rect.contains_point((to_point[0] - self.robot_buffer, to_point[1])) or
                    rect.contains_point((to_point[0], to_point[1] + self.robot_buffer)) or
                    rect.contains_point((to_point[0], to_point[1] - self.robot_buffer))):
                    return False
            elif shape == 'circle':
                radius = params[0]
                if (self.heuristic((x, y), from_point) <= radius + self.robot_buffer or 
                    self.heuristic((x, y), to_point) <= radius + self.robot_buffer):
                    return False
            elif shape == 'polygon':
                poly = Polygon(params)
                if (poly.contains_point((from_point[0], from_point[1])) or 
                    poly.contains_point((to_point[0], to_point[1])) or
                    poly.contains_point((from_point[0] + self.robot_buffer, from_point[1])) or
                    poly.contains_point((from_point[0] - self.robot_buffer, from_point[1])) or
                    poly.contains_point((from_point[0], from_point[1] + self.robot_buffer)) or
                    poly.contains_point((from_point[0], from_point[1] - self.robot_buffer)) or
                    poly.contains_point((to_point[0] + self.robot_buffer, to_point[1])) or
                    poly.contains_point((to_point[0] - self.robot_buffer, to_point[1])) or
                    poly.contains_point((to_point[0], to_point[1] + self.robot_buffer)) or
                    poly.contains_point((to_point[0], to_point[1] - self.robot_buffer))):
                    return False

        # Check for collision along the path
        num_checks = max(int(self.heuristic(from_point, to_point) / self.robot_buffer), 2)
        for i in range(1, num_checks):
            t = i / num_checks
            check_point = (int(from_point[0] + t * (to_point[0] - from_point[0])),
                           int(from_point[1] + t * (to_point[1] - from_point[1])))
            if self.point_in_collision(check_point):
                return False

        return True

    def extract_path(self, tree: Dict, end_node: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Extract the path from the tree structure used in RRT and RRT*.

        Args:
            tree (Dict): A dictionary representing the tree structure where keys are nodes and values are parent nodes.
            end_node (Tuple[int, int]): The end node of the path.

        Returns:
            List[Tuple[int, int]]: The extracted path from start to end_node.
        """
        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = tree[current]
        return path[::-1]

    def is_goal_reached(self, point: Tuple[int, int]) -> bool:
        """
        Check if a point is within the goal tolerance zone, considering the robot's buffer.

        Args:
            point (Tuple[int, int]): The point to check.

        Returns:
            bool: True if the point is within the goal tolerance zone, False otherwise.
        """
        return self.heuristic(point, self.goal) <= self.goal_tolerance + self.robot_buffer

    def point_in_collision(self, point: Tuple[int, int]) -> bool:
        """
        Check if a given point collides with any obstacle in the environment, considering the robot's buffer.

        Args:
            point (Tuple[int, int]): The point to check for collision.

        Returns:
            bool: True if the point is in collision with any obstacle, False otherwise.
        """
        buffer_points = [
            (point[0] + dx, point[1] + dy)
            for dx in [-self.robot_buffer, 0, self.robot_buffer]
            for dy in [-self.robot_buffer, 0, self.robot_buffer]
        ]

        for check_point in buffer_points:
            for obstacle in self.obstacles:
                x, y, shape, params = obstacle
                if shape == 'rectangle':
                    width, height = params
                    rect = Rectangle((x-width/2, y-height/2), width, height)
                    if rect.contains_point(check_point):
                        return True
                elif shape == 'circle':
                    radius = params[0]
                    if np.sqrt((check_point[0] - x)**2 + (check_point[1] - y)**2) <= radius:
                        return True
                elif shape == 'polygon':
                    poly = Polygon(params)
                    if poly.contains_point(check_point):
                        return True
        return False

def main():
    """
    Main function to demonstrate the usage of the Planner class.

    This function creates a Planner instance, sets up obstacles, start and goal points,
    defines a move set, and runs the A* algorithm with visualization.
    """
    planner = Planner()
    
    # Define obstacles
    obstacles = [
        (20, 20, 'rectangle', [10, 30]),  # Rectangle at (20, 20) with width 10 and height 30
        (50, 50, 'circle', [15]),         # Circle at (50, 50) with radius 15
        (80, 80, 'polygon', [(75, 75), (85, 75), (80, 85)])  # Triangle
    ]
    planner.set_obstacles(obstacles)
    
    # Set start and goal
    planner.set_start_goal((2, 2), (85, 75))
    
    # Define move set and step size
    move_set = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    step_size = 5
    planner.set_move_set(move_set, step_size)
    
    # Set robot buffer
    planner.set_robot_buffer(2)
    
    # Set goal tolerance
    planner.set_goal_tolerance(5)
    
    # Run A* algorithm
    path = planner.a_star(visualize=True)
    print("A* Path:", path)
    
    plt.title("A* Path Planning")
    plt.show()

if __name__ == "__main__":
    main()