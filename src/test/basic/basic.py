import unittest
import astar

class BasicTests(unittest.TestCase):

    def test_bestpath(self):
         """ensure that we take the shortest path, and not the path with less elements.
            the path with less elements is A -> B with a distance of 100
            the shortest path is A -> C -> D -> B with a distance of 60
         """
         nodes = {'A':[('B',100),('C',20)], 'C':[('D',20)], 'D':[('B',20)]}
         def neighbors(n):
             for n1, d in nodes[n]:
                 yield n1
         def distance(n1, n2):
             for n, d in nodes[n1]:
                 if n==n2:
                     return d
         def cost(n,goal):
             return 1
         path = list(astar.find_path('A', 'B', neighbors_fnct=neighbors, heuristic_cost_estimate_fnct=cost, distance_between_fnct=distance))
         self.assertEqual(4, len(path))
         for i, n in enumerate('ACDB'):
             self.assertEqual(n, path[i])

if __name__ == '__main__':
    unittest.main()
