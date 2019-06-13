# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

import logging
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop

#-------------------------------------------------------------------------------
# module metada

__author__ = "Julien Rialland"
__copyright__ = "Copyright 2012-2019, J.Rialland"
__license__ = "BSD"
__version__ = "0.95"
__maintainer__ = __author__
__email__ = ''.join(map(chr, [106, 117, 108, 105, 101, 110, 46, 114, 105,
                              97, 108, 108, 97, 110, 100, 64, 103, 109, 97, 105, 108, 46, 99, 111, 109]))
__status__ = "Production"

#-------------------------------------------------------------------------------
# module constants

Infinite = float('inf')

# module symbols (defined afterwards)
AStar = None
find_path = None

#-------------------------------------------------------------------------------
# implementation
class AStar_Py:
    """pure-python implementation of the algorithm"""

    __metaclass__ = ABCMeta
    __slots__ = ()

    class SearchNode:
        __slots__ = ('data', 'gscore', 'fscore',
                     'closed', 'came_from', 'out_openset')

        def __init__(self, data, gscore=Infinite, fscore=Infinite):
            self.data = data
            self.gscore = gscore
            self.fscore = fscore
            self.closed = False
            self.out_openset = True
            self.came_from = None

        def __lt__(self, b):
            return self.fscore < b.fscore

    class SearchNodeDict(dict):

        def __missing__(self, k):
            v = AStar.SearchNode(k)
            self.__setitem__(k, v)
            return v

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal):
        """Computes the estimated (rough) distance between a node and the goal, this method must be implemented in a subclass. The second parameter is always the goal."""
        raise NotImplementedError

    @abstractmethod
    def distance_between(self, n1, n2):
        """Gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
           n2 is guaranteed to belong to the list returned by the call to neighbors(n1).
           This method must be implemented in a subclass."""
        raise NotImplementedError

    @abstractmethod
    def neighbors(self, node):
        """For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedError

    def is_goal_reached(self, current, goal):
        """ returns true when we can consider that 'current' is the goal"""
        return current == goal

    def reconstruct_path(self, last, reversePath=False):
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from
        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(self, start, goal, reversePath=False):

        if self.is_goal_reached(start, goal):
            return [start]

        searchNodes = AStar.SearchNodeDict()

        startNode = searchNodes[start] = AStar.SearchNode(
            start, gscore=.0, fscore=self.heuristic_cost_estimate(start, goal))

        openSet = []
        heappush(openSet, startNode)
        while openSet:

            current = heappop(openSet)
            print("current " + str(current.data), current.fscore);

            if self.is_goal_reached(current.data, goal):
                print("goal is reached");
                return self.reconstruct_path(current, reversePath)

            current.out_openset = True
            current.closed = True

            for neighbor in [searchNodes[n] for n in self.neighbors(current.data)]:
                print("    " + str(neighbor.data))
                if neighbor.closed:
                    print("    is closed")
                    continue

                tentative_gscore = current.gscore + \
                    self.distance_between(current.data, neighbor.data)
                
                if tentative_gscore >= neighbor.gscore:
                    print("   tentative_gscore >= neighbor.gscore") 
                    continue

                neighbor.came_from = current
                neighbor.gscore = tentative_gscore
                neighbor.fscore = tentative_gscore + \
                    self.heuristic_cost_estimate(neighbor.data, goal)

                if neighbor.out_openset:
                    neighbor.out_openset = False
                    heappush(openSet, neighbor) 
                    print("    add to openset")
        return None



def find_path_Py(start, goal, neighbors_fnct, reversePath=False, heuristic_cost_estimate_fnct=lambda a, b: Infinite, distance_between_fnct=lambda a, b: 1.0, is_goal_reached_fnct=lambda a, b: a == b):
    """A non-class version of the path finding algorithm"""
    class FindPath(AStar):

        def heuristic_cost_estimate(self, current, goal):
            return heuristic_cost_estimate_fnct(current, goal)

        def distance_between(self, n1, n2):
            return distance_between_fnct(n1, n2)

        def neighbors(self, node):
            return neighbors_fnct(node)

        def is_goal_reached(self, current, goal):
            return is_goal_reached_fnct(current, goal)
    return FindPath().astar(start, goal, reversePath)

#define the 'Astar' class as being the pure-python implementation by default
AStar = AStar_Py
find_path = find_path_Py

#-------------------------------------------------------------------------------
# native implementation if available

#try to load the native module
try:
    import astar_native

    class Astar_Native_Param:
        """wraps the parameters that are needed by the C implementation"""
        def __init__(self, start, goal, neighbors_fn, reverse_path, heuristic_cost_estimate_fn, distance_between_fn, is_goal_reached_fn):
            self.start = start
            self.goal = goal
            self.neighbors_fn = neighbors_fn
            self.reverse_path = reverse_path
            self.heuristic_cost_estimate_fn = heuristic_cost_estimate_fn
            self.distance_between_fn = distance_between_fn
            self.is_goal_reached_fn = is_goal_reached_fn

    class Astar_Native(AStar_Py):
        """uses the C implementation"""
        def astar(self, start, goal, reversePath=False):
            param = Astar_Native_Param(
                start,
                goal,
                lambda a : self.neighbors(a),
                reversePath,
                lambda a,b : self.heuristic_cost_estimate(a, b),
                lambda a,b : self.distance_between(a, b),
                lambda a,b : self.is_goal_reached(a, b)
            )
            return astar_native.astar(param)

    def find_path_Native(start, goal, neighbors_fnct, reversePath=False, heuristic_cost_estimate_fnct=lambda a, b: Infinite, distance_between_fnct=lambda a, b: 1.0, is_goal_reached_fnct=lambda a, b: a == b):
        param = Astar_Native_Param(
                start,
                goal,
                neighbors_fnct,
                reversePath,
                heuristic_cost_estimate_fnct,
                distance_between_fnct,
                is_goal_reached_fnct
            )
        return astar_native.astar(param)

    #redefine the exported symbols
    AStar = Astar_Native
    find_path = find_path_Native
    logging.info('Using astar_native implementation')

except Exception:
    logging.exception('The astar_native module is not available - Pure python implementation will be used instead')
    pass

__all__ = ['AStar', 'find_path']

