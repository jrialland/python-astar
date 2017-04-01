import astar
import os
import sys
import csv
import math
from difflib import SequenceMatcher

class Station:
    def __init__(self, id, name, position):
        self.id = id
        self.name = name
        self.position = position
        self.links = []

def build_data():
    """builds the 'map' by reading the data files"""
    stations = {}
    rootdir = os.path.dirname(os.path.abspath(sys.argv[0]))
    r = csv.reader(open(os.path.join(rootdir, 'underground_stations.csv')))
    next(r)  # jump the first line
    for record in r:
        id = int(record[0])
        lat = float(record[1])
        lon = float(record[2])
        name = record[3]
        stations[id] = Station(id, name, (lat, lon))

    r = csv.reader(open(os.path.join(rootdir, 'underground_routes.csv')))
    next(r)  # jump the first line
    for id1, id2, lineNumber in r:
        id1 = int(id1)
        id2 = int(id2)
        stations[id1].links.append(stations[id2])
        stations[id2].links.append(stations[id1])
    return stations


def get_station_by_name(stations, name):
    """lookup by name, the name does not have to be exact."""
    name = name.lower()
    ratios = [(SequenceMatcher(None, name, v.name.lower()).ratio(), v)
              for v in stations.values()]
    best = max(ratios, key=lambda a: a[0])
    if best[0] > 0.7:
        return best[1]
    else:
        return None

def get_path(s1, s2):
    """ runs astar on the map"""

    def distance(n1, n2):
        """computes the distance between two stations"""
        latA, longA = n1.position
        latB, longB = n2.position
        # convert degres to radians!!
        latA, latB, longA, longB = map(
            lambda d: d * math.pi / 180, (latA, latB, longA, longB))
        x = (longB - longA) * math.cos((latA + latB) / 2)
        y = latB - latA
        return math.hypot(x, y)

    return astar.find_path(s1, s2, neighbors_fnct=lambda s: s.links, heuristic_cost_estimate_fnct=distance, distance_between_fnct=distance)

if __name__ == '__main__':

    if len(sys.argv) != 3:
        print(
            'Usage : {script} <station1> <station2>'.format(script=sys.argv[0]))
        sys.exit(1)

    stations = build_data()

    station1 = get_station_by_name(stations, sys.argv[1])
    station2 = get_station_by_name(stations, sys.argv[2])

    path = get_path(station1, station2)

    if path:
        for s in path:
            print(s.name)
    else:
        raise Exception('path not found!')

