#! /usr/bin/env python
# -*- coding: utf-8 -*-

""" generic A-Star path searching algorithm """

import sys
from abc import ABCMeta, abstractmethod

__author__ = "Julien Rialland"
__copyright__ = "Copyright 2012, J.Rialland"
__license__ = "BSD"
__version__ = "0.9"
__maintainer__ = __author__
__email__ = "julien.rialland@gmail.com"
__status__ = "Production"


class AStar:

    __metaclass__ = ABCMeta

    @abstractmethod
    def heuristic_cost_estimate(self, start, goal):
        """computes the estimated (rough) distance between two random nodes, this method must be implemented in a subclass"""
        raise NotImplementedException

    @abstractmethod
    def distance_between(self, n1, n2):
        """gives the real distance between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors), this method must be implemented in a subclass"""
        raise NotImplementedException

    @abstractmethod
    def neighbors(self, node):
        """for a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass"""
        raise NotImplementedException

    def _yield_path(self, came_from, last):
        yield last
        current = came_from[last]
        while True:
            yield current
            if current in came_from:
                current = came_from[current]
            else:
                break

    def _reconstruct_path(self, came_from, last):
        return list(reversed([p for p in self._yield_path(came_from, last)]))

    def astar(self, start, goal):
        """applies the a-star path searching algorithm in order to find a route between a 'start' node and a 'root' node"""
        closedset = set([])    # The set of nodes already evaluated.
        # The set of tentative nodes to be evaluated, initially containing the
        # start node
        openset = set([start])
        came_from = {}    # The map of navigated nodes.

        g_score = {}
        g_score[start] = 0   # Cost from start along best known path.

        # Estimated total cost from start to goal through y.
        f_score = {}
        f_score[start] = self.heuristic_cost_estimate(start, goal)

        while len(openset) > 0:
            # the node in openset having the lowest f_score[] value
            current = min(f_score, key=f_score.get)
            if current == goal:
                return self._reconstruct_path(came_from, goal)
            openset.discard(current)  # remove current from openset
            del f_score[current]
            closedset.add(current)  # add current to closedset

            for neighbor in self.neighbors(current):
                if neighbor in closedset:
                    continue
                tentative_g_score = g_score[
                    current] + self.distance_between(current, neighbor)
                if (neighbor not in openset) or (tentative_g_score < g_score[neighbor]):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + \
                        self.heuristic_cost_estimate(neighbor, goal)
                    openset.add(neighbor)
        return None

__all__ = ['AStar']

if __name__ == '__main__':
    import sys
    import math

    def make_maze(w=30, h=30):
        """returns an ascii maze as a string"""
        from random import shuffle, randrange
        vis = [[0] * w + [1] for _ in range(h)] + [[1] * (w + 1)]
        ver = [["|  "] * w + ['|'] for _ in range(h)] + [[]]
        hor = [["+--"] * w + ['+'] for _ in range(h + 1)]

        def walk(x, y):
            vis[y][x] = 1

            d = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
            shuffle(d)
            for (xx, yy) in d:
                if vis[yy][xx]:
                    continue
                if xx == x:
                    hor[max(y, yy)][x] = "+  "
                if yy == y:
                    ver[y][max(x, xx)] = "   "
                walk(xx, yy)

        walk(randrange(w), randrange(h))
        result = ''
        for (a, b) in zip(hor, ver):
            result = result + (''.join(a + ['\n'] + b)) + '\n'
        return result.strip()

    def drawmaze(maze, set1=[], set2=[], c='#', c2='*'):
        """returns an ascii maze, drawing eventually one (or 2) sets of positions.
            useful to draw the solution found by the astar algorithm
        """
        set1 = list(set1)
        set2 = list(set2)
        lines = maze.strip().split('\n')
        width = len(lines[0])
        height = len(lines)
        result = ''
        for j in range(height):
            for i in range(width):
                if (i, j) in set1:
                    result = result + c
                elif (i, j) in set2:
                    result = result + c2
                else:
                    result = result + lines[j][i]
            result = result + '\n'
        return result

    class MazeSolver(AStar):

        """sample use of the astar algorithm. In this exemple we work on a maze made of ascii characters,
        and a 'node' is just a (x,y) tuple that represents a reachable position"""

        def __init__(self, maze):
            self.lines = maze.strip().split('\n')
            self.width = len(self.lines[0])
            self.height = len(self.lines)

        def heuristic_cost_estimate(self, n1, n2):
            """computes the 'direct' distance between two (x,y) tuples"""
            (x1, y1) = n1
            (x2, y2) = n2
            return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

        def distance_between(self, n1, n2):
            """this method always returns 1, as two 'neighbors' are always adajcent"""
            return 1

        def neighbors(self, node):
            """ for a given coordinate in the maze, returns up to 4 adjacent nodes that can be reached (=any adjacent coordinate that is not a wall)
            """
            x, y = node
            for i, j in [(0, -1), (0, +1), (-1, 0), (+1, 0)]:
                x1 = x + i
                y1 = y + j
                if x1 > 0 and y1 > 0 and x1 < self.width and y1 < self.height:
                    if self.lines[y1][x1] == ' ':
                        yield (x1, y1)

    # make a big maze
    size = 20
    m = make_maze(size, size)

    # what is the size of it?
    w = len(m.split('\n')[0])
    h = len(m.split('\n'))

    start = (1, 1)  # we start at the upper left corner
    goal = (w - 2, h - 2)  # we want to reach the lower right corner

    # let's solve it
    print(drawmaze(m, MazeSolver(m).astar(start, goal)))
