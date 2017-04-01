import sys, os
import math
import unittest
import astar

class Station:

    def __init__(self, id, nom, position):
        self.id = id
        self.nom = nom
        self.position = position
        self.liaisons = []


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
        y = (latB - latA)
        d = math.sqrt(x**2 + y**2) * 6371
        return d

    def neighbors(self, node):
        return node.liaisons


def solve_tan(in_stream):
    stations = {}
    id_depart = in_stream.readline().strip()
    id_arrivee = in_stream.readline().strip()
    N = int(in_stream.readline())
    for _ in range(N):
        items = in_stream.readline().strip().split(',')
        id = items[0]
        nom = items[1].replace('"', '')
        position = float(items[3]), float(items[4])
        stations[id] = Station(id, nom, position)

    M = int(in_stream.readline())
    for _ in range(M):
        s1, s2 = in_stream.readline().strip().split()
        stations[s1].liaisons.append(stations[s2])

    depart = stations[id_depart]
    arrive = stations[id_arrivee]
    chemin = TanFinder().astar(depart, arrive)

    if chemin is None:
        return 'IMPOSSIBLE\n'
    else:
        return ''.join([station.nom + '\n' for station in chemin])


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
