import heapq
import numpy as np
import random
from typing import List, Tuple, Dict
import matplotlib.pyplot as plt

class Planner:
    def __init__(self):
        self.graph = {}  # For graph-based algorithms
        self.obstacles = []  # For sampling-based algorithms
        self.start = None
        self.goal = None
        self.fig = None
        self.ax = None

    def set_graph(self, graph: Dict):
        self.graph = graph

    def set_obstacles(self, obstacles: List[Tuple[float, float]]):
        self.obstacles = obstacles

    def set_start_goal(self, start: Tuple[float, float], goal: Tuple[float, float]):
        self.start = start
        self.goal = goal

    def heuristic(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return np.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

    def initialize_plot(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)
        self.ax.plot(*zip(*self.obstacles), 'ko', markersize=5)
        self.ax.plot(*self.start, 'go', markersize=10)
        self.ax.plot(*self.goal, 'ro', markersize=10)

    def update_plot(self, current, neighbors=None):
        if neighbors:
            for neighbor in neighbors:
                self.ax.plot([current[0], neighbor[0]], [current[1], neighbor[1]], 'y-', linewidth=0.5)
        self.ax.plot(*current, 'bo', markersize=3)
        plt.pause(0.001)

    def plot_final_path(self, path):
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
        return (random.uniform(0, 1), random.uniform(0, 1))  # Assuming a 1x1 space

    def steer(self, from_point: Tuple[float, float], to_point: Tuple[float, float], step_size: float) -> Tuple[float, float]:
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
        # Simplified collision check. In a real scenario, this would be more complex.
        for obstacle in self.obstacles:
            if self.heuristic(obstacle, to_point) < 0.1:  # Assuming obstacles are points with some radius
                return False
        return True

    def extract_path(self, tree: Dict, end_node: Tuple[float, float]) -> List[Tuple[float, float]]:
        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = tree[current]
        return path[::-1]

def main():
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