import unittest
from python.planner import Planner

class TestPlanner(unittest.TestCase):
    """Test suite for the Planner class."""

    def setUp(self):
        """Set up the test environment before each test."""
        self.planner = Planner()

    def test_initialization(self):
        """Test that the Planner constructor creates a valid instance."""
        self.assertIsInstance(self.planner, Planner)

    def test_set_obstacles(self):
        """Test that setting obstacles works correctly."""
        obstacles = [(1.0, 1.0), (2.0, 2.0)]
        self.planner.set_obstacles(obstacles)
        # Assuming there's a way to get the current obstacles
        self.assertEqual(self.planner.obstacles, obstacles)

    def test_a_star_path(self):
        """Test that A* algorithm returns a valid path."""
        start = (0.0, 0.0)
        goal = (0.9, 0.9)
        self.planner.set_start_goal(start, goal)
        
        # Create a simple graph for testing
        graph = {
            (0.0, 0.0): [(0.1, 0.1), (0.1, 0.0)],
            (0.1, 0.1): [(0.0, 0.0), (0.2, 0.2)],
            (0.2, 0.2): [(0.1, 0.1), (0.3, 0.3)],
            (0.3, 0.3): [(0.2, 0.2), (0.4, 0.4)],
            (0.4, 0.4): [(0.3, 0.3), (0.5, 0.5)],
            (0.5, 0.5): [(0.4, 0.4), (0.6, 0.6)],
            (0.6, 0.6): [(0.5, 0.5), (0.7, 0.7)],
            (0.7, 0.7): [(0.6, 0.6), (0.8, 0.8)],
            (0.8, 0.8): [(0.7, 0.7), (0.9, 0.9)],
            (0.9, 0.9): [(0.8, 0.8)]
        }
        self.planner.set_graph(graph)
        
        path = self.planner.a_star()
        
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_rrt_path_with_obstacles(self):
        """Test that RRT algorithm returns a valid path that avoids obstacles."""
        start = (0.1, 0.1)
        goal = (0.9, 0.9)
        self.planner.set_start_goal(start, goal)
        
        obstacles = [(0.5, 0.5), (0.6, 0.6)]
        self.planner.set_obstacles(obstacles)
        
        path = self.planner.rrt()
        
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 0)
        self.assertEqual(path[0], start)
        self.assertAlmostEqual(path[-1][0], goal[0], delta=0.1)
        self.assertAlmostEqual(path[-1][1], goal[1], delta=0.1)
        for obstacle in obstacles:
            for point in path:
                self.assertGreater(self.planner.heuristic(point, obstacle), 0.1)

if __name__ == '__main__':
    unittest.main()


