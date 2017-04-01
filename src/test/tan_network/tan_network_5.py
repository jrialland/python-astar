import sys, os
import math
import unittest
import astar

class Station:

    def __init__(self, id, name, position):
        self.id = id
        self.name = name
        self.position = position
        self.links = []


class TanFinder(astar.AStar):

    def heuristic_cost_estimate(self, current, goal):
        return self.distance_between(current, goal)

    def distance_between(self, n1, n2):
        latA, longA = n1.position
        latB, longB = n2.position

        # convert degres to radians!!
        latA, latB, longA, longB = map(
            lambda d: d * math.pi / 180, (latA, latB, longA, longB))
            
        x = (longB - longA) * math.cos((latA + latB) / 2)
        y = latB - latA
        return math.hypot(x, y)

    def neighbors(self, node):
        return node.links


def solve_tan(in_stream):
    stations = {}
    id_start = in_stream.readline().strip()
    id_goal = in_stream.readline().strip()
    N = int(in_stream.readline())
    for _ in range(N):
        items = in_stream.readline().strip().split(',')
        id = items[0]
        name = items[1].replace('"', '')
        position = float(items[3]), float(items[4])
        stations[id] = Station(id, name, position)

    M = int(in_stream.readline())
    for _ in range(M):
        s1, s2 = in_stream.readline().strip().split()
        stations[s1].links.append(stations[s2])

    start = stations[id_start]
    goal = stations[id_goal]
    path = TanFinder().astar(start, goal)

    if path is None:
        return 'IMPOSSIBLE\n'
    else:
        return ''.join([station.name + '\n' for station in path])


class TanFinderTest(unittest.TestCase):

    def test_solveTan(self):
        self.maxDiff=None
        rootdir = os.path.dirname(os.path.abspath(sys.argv[0]))
        with open(os.path.join(rootdir, 'tan_network_5.in.txt')) as inputFile:
            with open(os.path.join(rootdir, 'tan_network_5.out.txt')) as outputFile:
                output_data = outputFile.read()
                self.assertEqual(output_data, solve_tan(inputFile))

if __name__ == '__main__':
    unittest.main()
