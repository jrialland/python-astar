import unittest
from astar import AStar

class BasicAStar(AStar):
    def __init__(self, nodes):
        self.nodes = nodes

    def neighbors(self, n):
        for n1, d in self.nodes[n]:
            yield n1

    def distance_between(self, n1, n2):
        for n, d in self.nodes[n1]:
            if n == n2:
                return d
            
    def heuristic_cost_estimate(self, current, goal):
        return 1
    
    def is_goal_reached(self, current, goal):
        return current == goal

class BasicTests(unittest.TestCase):

    def test_bestpath(self):
        """ensure that we take the shortest path, and not the path with less elements.
           the path with less elements is A -> B with a distance of 100
           the shortest path is A -> C -> D -> B with a distance of 60
        """
        nodes = {'A': [('B', 100), ('C', 20)],
                 'C': [('D', 20)], 
                 'D': [('B', 20)]}

        path = BasicAStar(nodes).astar('A', 'B')
        self.assertIsNotNone(path)
        if path:
            path = list(path)
            self.assertEqual(4, len(path))
            for i, n in enumerate('ACDB'):
                self.assertEqual(n, path[i])

    def test_issue_15(self):
        """This test case reproduces https://github.com/jrialland/python-astar/issues/15.
        B has no neighbors, therefore the computation should return None and not raise an exception.
        """
        node = {
            'A': [('B', 200000)],
            'C': [('D', 200000)],
            'D': [('E', 200000)],
            'E': [('F', 200000)],
            'B': [],
            'F': []
        }
        path = BasicAStar(node).astar('A', 'D')
        self.assertIsNone(path)

if __name__ == '__main__':
    unittest.main()
