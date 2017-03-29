#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" generic A-Star path searching algorithm """

import sys
from abc import ABCMeta, abstractmethod
from heapq import heappush, heappop, heapify

__author__ = "Julien Rialland"
__copyright__ = "Copyright 2012-2017, J.Rialland"
__license__ = "BSD"
__version__ = "0.9"
__maintainer__ = __author__
__email__ = "julien.rialland@gmail.com"
__status__ = "Production"


class SearchNode:
    __slots__ = ('data', 'gscore', 'fscore', 'closed', 'came_from', 'in_by_fscores')

    def __init__(self, data, gscore=0):
        self.data = data
        self.gscore = gscore
        self.fscore = 0
        self.closed = False
        self.came_from = None
        self.in_by_fscores = False

    def __lt__(self, b):
        return self.fscore < b.fscore


class AStar:

    __metaclass__ = ABCMeta

    @abstractmethod
    def heuristic_cost_estimate(self, start, goal):
        """Computes the estimated (rough) distance between two random nodes, this method must be implemented in a subclass"""
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
        return current == goal

    def reconstruct_path(self, last):
        path = []
        current = last
        while current:
            path.append(current.data)
            current = current.came_from
        return reversed(path)

    def astar(self, start, goal):
        """applies the a-star path searching algorithm in order to find a route between a 'start' node and a 'goal' node"""

        class AutoDict(dict):
            """A dictionary that creates new entries when a key is missing"""
            def __missing__(self, k):
                s = SearchNode(data=k)
                self.__setitem__(k, s)
                return s

        searchNodes = AutoDict()

        goalNode = searchNodes[goal] = SearchNode(goal)
        startNode = searchNodes[start] = SearchNode(data=start, gscore=0)

        # Estimated total cost from start to goal.
        startNode.fscore = self.heuristic_cost_estimate(
            startNode.data, goalNode.data)

        #priority queue that stores the nodes to be tested
        by_fscores = []
        heappush(by_fscores, startNode)
        startNode.in_by_fscores = True

        while by_fscores:
            current = heappop(by_fscores)
            current.in_by_fscores = False
            current.closed = True
            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current)
            for neighbor in [searchNodes[n] for n in self.neighbors(current.data)]:
                if neighbor.closed:
                    continue
                tentative_g_score = current.gscore + \
                    self.distance_between(current.data, neighbor.data)
                if not neighbor.in_by_fscores or tentative_g_score < neighbor.gscore:
                    neighbor.came_from = current
                    neighbor.gscore = tentative_g_score
                    neighbor.fscore = tentative_g_score + \
                        self.heuristic_cost_estimate(neighbor.data, goal)
                    neighbor.in_by_fscores = True
                    heappush(by_fscores, neighbor)
        return None

__all__ = ['AStar']

