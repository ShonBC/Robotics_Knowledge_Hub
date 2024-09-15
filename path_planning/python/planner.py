import heapq
import numpy as np
import random
from typing import List, Tuple, Dict
import matplotlib.pyplot as plt

class Planner:
    """
    A class for implementing various path planning algorithms.

    This class provides methods for different path planning algorithms including A*, Weighted A*,
    Dijkstra's algorithm, RRT (Rapidly-exploring Random Tree), and RRT* (Optimal RRT).
    It also includes visualization capabilities for the path planning process.

    Attributes:
        graph (Dict): A dictionary representing the graph for graph-based algorithms.
        obstacles (List[Tuple[float, float]]): A list of obstacle coordinates for sampling-based algorithms.
        start (Tuple[float, float]): The start point coordinates.
        goal (Tuple[float, float]): The goal point coordinates.
        fig (matplotlib.figure.Figure): The matplotlib figure for visualization.
        ax (matplotlib.axes.Axes): The matplotlib axes for plotting.

    Methods:
        set_graph(graph: Dict): Sets the graph for graph-based algorithms.
        set_obstacles(obstacles: List[Tuple[float, float]]): Sets the obstacles for sampling-based algorithms.
        set_start_goal(start: Tuple[float, float], goal: Tuple[float, float]): Sets the start and goal points.
        heuristic(a: Tuple[float, float], b: Tuple[float, float]) -> float: Calculates the heuristic distance between two points.
        initialize_plot(): Initializes the plot for visualization.
        update_plot(current, neighbors=None): Updates the plot during the path planning process.
        plot_final_path(path): Plots the final path found by the algorithm.
        a_star(visualize: bool = False) -> List[Tuple[float, float]]: Implements the A* algorithm.
        weighted_a_star(weight: float = 1.5, visualize: bool = False) -> List[Tuple[float, float]]: Implements the Weighted A* algorithm.
        dijkstra(visualize: bool = False) -> List[Tuple[float, float]]: Implements Dijkstra's algorithm.
        rrt(max_iterations: int = 1000, step_size: float = 0.1, visualize: bool = False) -> List[Tuple[float, float]]: Implements the RRT algorithm.
        rrt_star(max_iterations: int = 1000, step_size: float = 0.1, search_radius: float = 0.5, visualize: bool = False) -> List[Tuple[float, float]]: Implements the RRT* algorithm.
        slam(visualize: bool = False): Placeholder for SLAM implementation.
        random_state() -> Tuple[float, float]: Generates a random state within the planning space.
        steer(from_point: Tuple[float, float], to_point: Tuple[float, float], step_size: float) -> Tuple[float, float]: Steers from one point towards another with a maximum step size.
        collision_free(from_point: Tuple[float, float], to_point: Tuple[float, float]) -> bool: Checks if a path between two points is collision-free.
        extract_path(tree: Dict, end_node: Tuple[float, float]) -> List[Tuple[float, float]]: Extracts the path from the tree structure used in RRT and RRT*.
    """

    def __init__(self):
        self.graph = {}  # For graph-based algorithms
        self.obstacles = []  # For sampling-based algorithms
        self.start = None
        self.goal = None
        self.fig = None
        self.ax = None

    def set_graph(self, graph: Dict):
        """
        Sets the graph for graph-based algorithms.

        Args:
            graph (Dict): A dictionary representing the graph where keys are nodes and values are lists of neighboring nodes.
        """
        self.graph = graph

    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        """
        Sets the obstacles for sampling-based algorithms.

        Args:
            obstacles (List[Tuple[float, float]]): A list of obstacle coordinates.
        """
        self.obstacles = obstacles

    def set_start_goal(self, start: Tuple[float, float], goal: Tuple[float, float]):
        """
        Sets the start and goal points for the path planning problem.

        Args:
            start (Tuple[float, float]): The start point coordinates.
            goal (Tuple[float, float]): The goal point coordinates.
        """
        self.start = start
        self.goal = goal

    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """
        Calculates the heuristic distance between two points.

        Args:
            a (Tuple[float, float]): The first point.
            b (Tuple[float, float]): The second point.

        Returns:
            float: The Euclidean distance between the two points.
        """
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def initialize_plot(self):
        """
        Initializes the plot for visualization.

        This method sets up the matplotlib figure and axes, and plots the obstacles, start, and goal points.
        """
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.plot(*zip(*self.obstacles), 'ko', markersize=5)
        self.ax.plot(*self.start, 'go', markersize=10)
        self.ax.plot(*self.goal, 'ro', markersize=10)

    def update_plot(self, current, neighbors=None):
        """
        Updates the plot during the path planning process.

        Args:
            current (Tuple[float, float]): The current node being explored.
            neighbors (List[Tuple[float, float]], optional): The neighbors of the current node. Defaults to None.
        """
        if neighbors:
            for neighbor in neighbors:
                self.ax.plot([current[0], neighbor[0]], [current[1], neighbor[1]], 'y-', linewidth=0.5)
        self.ax.plot(*current, 'bo', markersize=3)
        plt.pause(0.001)

    def plot_final_path(self, path):
        """
        Plots the final path found by the algorithm.

        Args:
            path (List[Tuple[float, float]]): The final path as a list of coordinate tuples.
        """
        path_x, path_y = zip(*path)
        self.ax.plot(path_x, path_y, 'g-', linewidth=4)
        plt.pause(0.001)

    def a_star(self, visualize: bool = False) -> List[Tuple[float, float]]:
        """
        Performs A* search algorithm to find the optimal path from start to goal.

        The A* algorithm uses a best-first search and finds a least-cost path from a given initial node to the goal node. 
        It uses a heuristic to estimate the cost of the cheapest path from any node to the goal.

        Args:
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[float, float]]: The optimal path from start to goal as a list of coordinate tuples.
                                       Returns an empty list if no path is found.

        Note:
            - This method assumes that self.start, self.goal, and self.graph have been properly set.
            - The heuristic function used is Euclidean distance.
            - Visualization, if enabled, shows the explored nodes and the final path.
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
                self.update_plot(current, self.graph[current])

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for neighbor in self.graph[current]:
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def weighted_a_star(self, weight: float = 1.5, visualize: bool = False) -> List[Tuple[float, float]]:
        """
        Performs Weighted A* search algorithm to find a path from start to goal.

        The Weighted A* algorithm is a variant of A* that allows for a trade-off between optimality and speed
        by inflating the heuristic value with a weight greater than 1.

        Args:
            weight (float): The weight to apply to the heuristic. Defaults to 1.5.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[float, float]]: The path from start to goal as a list of coordinate tuples.
                                       Returns an empty list if no path is found.

        Note:
            - This method assumes that self.start, self.goal, and self.graph have been properly set.
            - The heuristic function used is Euclidean distance, multiplied by the weight.
            - Visualization, if enabled, shows the explored nodes and the final path.
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
                self.update_plot(current, self.graph[current])

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for neighbor in self.graph[current]:
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + weight * self.heuristic(neighbor, self.goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def dijkstra(self, visualize: bool = False) -> List[Tuple[float, float]]:
        """
        Performs Dijkstra's algorithm to find the shortest path from start to goal.

        Dijkstra's algorithm is a graph search algorithm that finds the shortest path between nodes in a graph.

        Args:
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[float, float]]: The shortest path from start to goal as a list of coordinate tuples.
                                       Returns an empty list if no path is found.

        Note:
            - This method assumes that self.start, self.goal, and self.graph have been properly set.
            - Visualization, if enabled, shows the explored nodes and the final path.
        """
        if visualize:
            self.initialize_plot()

        open_set = [(0, self.start)]
        came_from = {}
        g_score = {self.start: 0}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if visualize:
                self.update_plot(current, self.graph[current])

            if current == self.goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(self.start)
                path = path[::-1]
                if visualize:
                    self.plot_final_path(path)
                return path

            for neighbor in self.graph[current]:
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor)
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    heapq.heappush(open_set, (g_score[neighbor], neighbor))

        return []

    def rrt(self, max_iterations: int = 1000, step_size: float = 0.1, visualize: bool = False) -> List[Tuple[float, float]]:
        """
        Performs Rapidly-exploring Random Tree (RRT) algorithm to find a path from start to goal.

        RRT is a sampling-based algorithm that builds a space-filling tree to explore the state space.

        Args:
            max_iterations (int): The maximum number of iterations to perform. Defaults to 1000.
            step_size (float): The maximum distance to extend the tree in each iteration. Defaults to 0.1.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[float, float]]: The path from start to goal as a list of coordinate tuples.
                                       Returns an empty list if no path is found.

        Note:
            - This method assumes that self.start, self.goal, and self.obstacles have been properly set.
            - Visualization, if enabled, shows the tree growth and the final path.
        """
        if visualize:
            self.initialize_plot()

        tree = {self.start: None}
        
        for _ in range(max_iterations):
            random_point = self.random_state()
            nearest_node = min(tree, key=lambda n: self.heuristic(n, random_point))
            
            new_node = self.steer(nearest_node, random_point, step_size)
            if self.collision_free(nearest_node, new_node):
                tree[new_node] = nearest_node
                
                if visualize:
                    self.update_plot(new_node)
                    self.ax.plot([nearest_node[0], new_node[0]], [nearest_node[1], new_node[1]], 'g-')

                if self.heuristic(new_node, self.goal) < step_size:
                    path = self.extract_path(tree, new_node)
                    if visualize:
                        self.plot_final_path(path)
                    return path
        
        return []

    def rrt_star(self, max_iterations: int = 1000, step_size: float = 0.1, search_radius: float = 0.5, visualize: bool = False) -> List[Tuple[float, float]]:
        """
        Performs RRT* (Optimal Rapidly-exploring Random Tree) algorithm to find an optimal path from start to goal.

        RRT* is an asymptotically optimal variant of RRT that continues to optimize the path as the number of samples increases.

        Args:
            max_iterations (int): The maximum number of iterations to perform. Defaults to 1000.
            step_size (float): The maximum distance to extend the tree in each iteration. Defaults to 0.1.
            search_radius (float): The radius within which nodes are considered for rewiring. Defaults to 0.5.
            visualize (bool): If True, visualizes the search process. Defaults to False.

        Returns:
            List[Tuple[float, float]]: The optimal path from start to goal as a list of coordinate tuples.
                                       Returns an empty list if no path is found.

        Note:
            - This method assumes that self.start, self.goal, and self.obstacles have been properly set.
            - Visualization, if enabled, shows the tree growth and the final path.
        """
        if visualize:
            self.initialize_plot()

        tree = {self.start: None}
        costs = {self.start: 0}
        
        for _ in range(max_iterations):
            random_point = self.random_state()
            nearest_node = min(tree, key=lambda n: self.heuristic(n, random_point))
            
            new_node = self.steer(nearest_node, random_point, step_size)
            if self.collision_free(nearest_node, new_node):
                near_nodes = [n for n in tree if self.heuristic(n, new_node) < search_radius]
                tree[new_node] = nearest_node
                costs[new_node] = costs[nearest_node] + self.heuristic(nearest_node, new_node)
                
                if visualize:
                    self.update_plot(new_node)
                    self.ax.plot([nearest_node[0], new_node[0]], [nearest_node[1], new_node[1]], 'g-')

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
                                self.ax.plot([new_node[0], near_node[0]], [new_node[1], near_node[1]], 'g-')
                
                if self.heuristic(new_node, self.goal) < step_size:
                    path = self.extract_path(tree, new_node)
                    if visualize:
                        self.plot_final_path(path)
                    return path
        
        return []

    def slam(self, visualize: bool = False):
        # SLAM is a complex algorithm that typically involves sensor data processing,
        # landmark detection, and map building. This is a placeholder for a real SLAM implementation.
        print("SLAM algorithm not implemented. It requires sensor data and more complex processing.")
        if visualize:
            print("Visualization for SLAM would typically show the robot's estimated position and the map being built.")
        return None

    def random_state(self) -> Tuple[float, float]:
        """
        Generate a random state within the planning space.

        Returns:
            Tuple[float, float]: A tuple representing a random (x, y) coordinate within a 1x1 space.
        """
        return (random.uniform(0, 1), random.uniform(0, 1))  # Assuming a 1x1 space

    def steer(self, from_point: Tuple[float, float], to_point: Tuple[float, float], step_size: float) -> Tuple[float, float]:
        """
        Steer from one point towards another with a maximum step size.

        Args:
            from_point (Tuple[float, float]): The starting point.
            to_point (Tuple[float, float]): The target point to steer towards.
            step_size (float): The maximum distance to move.

        Returns:
            Tuple[float, float]: The new point after steering.
        """
        dx = to_point[0] - from_point[0]
        dy = to_point[1] - from_point[1]
        distance = np.sqrt(dx**2 + dy**2)
        if distance < step_size:
            return to_point
        else:
            theta = np.arctan2(dy, dx)
            return (from_point[0] + step_size * np.cos(theta),
                    from_point[1] + step_size * np.sin(theta))

    def collision_free(self, from_point: Tuple[float, float], to_point: Tuple[float, float]) -> bool:
        """
        Check if a path between two points is collision-free.

        Args:
            from_point (Tuple[float, float]): The starting point of the path.
            to_point (Tuple[float, float]): The ending point of the path.

        Returns:
            bool: True if the path is collision-free, False otherwise.

        Note:
            This is a simplified collision check. In a real scenario, this would be more complex.
        """
        # Simplified collision check. In a real scenario, this would be more complex.
        for obstacle in self.obstacles:
            if self.heuristic(obstacle, to_point) < 0.1:  # Assuming obstacles are points with some radius
                return False
        return True

    def extract_path(self, tree: Dict, end_node: Tuple[float, float]) -> List[Tuple[float, float]]:
        """
        Extract the path from the tree structure used in RRT and RRT*.

        Args:
            tree (Dict): A dictionary representing the tree structure where keys are nodes and values are parent nodes.
            end_node (Tuple[float, float]): The end node of the path.

        Returns:
            List[Tuple[float, float]]: The extracted path from start to end_node.
        """
        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = tree[current]
        return path[::-1]

def main():
    """
    Main function to demonstrate the usage of the Planner class.

    This function creates a Planner instance, sets up a graph, start and goal points,
    adds obstacles, and runs the A* algorithm with visualization.
    """
    planner = Planner()
    # Example usage
    # Create a larger, more interesting graph
    graph = {}
    for x in range(10):
        for y in range(10):
            neighbors = []
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < 10 and 0 <= ny < 10:
                    if random.random() > 0.3:  # 70% chance of connection
                        neighbors.append((nx/10, ny/10))
            graph[(x/10, y/10)] = neighbors

    planner.set_graph(graph)
    
    # Set start and goal to opposite corners
    planner.set_start_goal((0, 0), (0.9, 0.9))
    
    # Add some obstacles
    obstacles = [(random.uniform(0.1, 0.8), random.uniform(0.1, 0.8)) for _ in range(10)]
    planner.set_obstacles(obstacles)
    
    path = planner.a_star(visualize=True)
    print("A* Path:", path)
    
    plt.title("A* Path Planning")
    plt.show()

if __name__ == "__main__":
    main()