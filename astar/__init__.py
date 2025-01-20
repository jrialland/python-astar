# -*- coding: utf-8 -*-
""" generic A-Star path searching algorithm """

from abc import ABC, abstractmethod
from typing import Callable, Dict, Iterable, Union, TypeVar, Generic
from math import inf as infinity
from operator import attrgetter
import heapq

# introduce generic type
T = TypeVar("T")


################################################################################
class SearchNode(Generic[T]):
    """Representation of a search node"""

    __slots__ = ("data", "gscore", "fscore", "closed", "came_from", "in_openset", "cache")

    def __init__(
        self, data: T, gscore: float = infinity, fscore: float = infinity
    ) -> None:
        self.data = data
        self.gscore = gscore
        self.fscore = fscore
        self.closed = False
        self.in_openset = False
        self.came_from: Union[None, SearchNode[T]] = None
        self.cache: Any = None

    def __lt__(self, b: "SearchNode[T]") -> bool:
        """Natural order is based on the fscore value & is used by heapq operations"""
        return self.fscore < b.fscore


################################################################################
class SearchNodeDict(Dict[T, SearchNode[T]]):
    """A dict that returns a new SearchNode when a key is missing"""

    def __missing__(self, k) -> SearchNode[T]:
        v = SearchNode(k)
        self.__setitem__(k, v)
        return v


################################################################################
SNType = TypeVar("SNType", bound=SearchNode)


class OpenSet(Generic[SNType]):
    def __init__(self) -> None:
        self.heap: list[SNType] = []

    def push(self, item: SNType) -> None:
        item.in_openset = True
        heapq.heappush(self.heap, item)

    def pop(self) -> SNType:
        item = heapq.heappop(self.heap)
        item.in_openset = False
        return item

    def remove(self, item: SNType) -> None:
        idx = self.heap.index(item)
        item.in_openset = False
        item = self.heap.pop()
        if idx < len(self.heap):
            self.heap[idx] = item
            # Fix heap invariants
            heapq._siftup(self.heap, idx)
            heapq._siftdown(self.heap, 0, idx)

    def __len__(self) -> int:
        return len(self.heap)


################################################################################*


class AStar(ABC, Generic[T]):
    __slots__ = ()

    @abstractmethod
    def heuristic_cost_estimate(self, current: T, goal: T) -> float:
        """
        Computes the estimated (rough) distance between a node and the goal.
        The second parameter is always the goal.

        This method must be implemented in a subclass.
        """
        raise NotImplementedError

    def distance_between(self, n1: T, n2: T) -> float:
        """
        Gives the real distance between two adjacent nodes n1 and n2 (i.e n2
        belongs to the list of n1's neighbors).
        n2 is guaranteed to belong to the list returned by the call to neighbors(n1).

        This method (or "path_distance_between") must be implemented in a subclass.
        """
        raise NotImplementedError

    def path_distance_between(self, n1: SearchNode[T], n2: SearchNode[T]) -> float:
        """
        Gives the real distance between the node n1 and its neighbor n2.
        n2 is guaranteed to belong to the list returned by the call to
        path_neighbors(n1).

        Calls "distance_between"`by default.
        """
        return self.distance_between(n1.data, n2.data)

    def neighbors(self, node: T) -> Iterable[T]:
        """
        For a given node, returns (or yields) the list of its neighbors.

        This method (or "path_neighbors") must be implemented in a subclass.
        """
        raise NotImplementedError

    def path_neighbors(self, node: SearchNode[T]) -> Iterable[T]:
        """
        For a given node, returns (or yields) the list of its reachable neighbors.
        Calls "neighbors" by default.
        """
        return self.neighbors(node.data)

    def _neighbors(self, current: SearchNode[T], search_nodes: SearchNodeDict[T]) -> Iterable[SearchNode]:
        return (search_nodes[n] for n in self.path_neighbors(current))

    def is_goal_reached(self, current: T, goal: T) -> bool:
        """
        Returns true when we can consider that 'current' is the goal.
        The default implementation simply compares `current == goal`, but this
        method can be overwritten in a subclass to provide more refined checks.
        """
        return current == goal

    def reconstruct_path(self, last: SearchNode, reversePath=False) -> Iterable[T]:
        def _gen():
            current = last
            while current:
                yield current.data
                current = current.came_from

        if reversePath:
            return _gen()
        else:
            return reversed(list(_gen()))

    def astar(
        self, start: T, goal: T, reversePath: bool = False
    ) -> Union[Iterable[T], None]:
        if self.is_goal_reached(start, goal):
            return [start]

        openSet: OpenSet[SearchNode[T]] = OpenSet()
        searchNodes: SearchNodeDict[T] = SearchNodeDict()
        startNode = searchNodes[start] = SearchNode(
            start, gscore=0.0, fscore=self.heuristic_cost_estimate(start, goal)
        )
        openSet.push(startNode)

        while openSet:
            current = openSet.pop()

            if self.is_goal_reached(current.data, goal):
                return self.reconstruct_path(current, reversePath)

            current.closed = True

            for neighbor in self._neighbors(current, searchNodes):
                if neighbor.closed:
                    continue

                gscore = current.gscore + self.path_distance_between(current, neighbor)

                if gscore >= neighbor.gscore:
                    continue

                fscore = gscore + self.heuristic_cost_estimate(
                    neighbor.data, goal
                )

                if neighbor.in_openset:
                    if neighbor.fscore < fscore:
                        # the new path to this node isn't better
                        continue

                    # we have to remove the item from the heap, as its score has changed
                    openSet.remove(neighbor)

                # update the node
                neighbor.came_from = current
                neighbor.gscore = gscore
                neighbor.fscore = fscore

                openSet.push(neighbor)

        return None


################################################################################
U = TypeVar("U")


def find_path(
    start: U,
    goal: U,
    neighbors_fnct: Callable[[U], Iterable[U]],
    reversePath=False,
    heuristic_cost_estimate_fnct: Callable[[U, U], float] = lambda a, b: infinity,
    distance_between_fnct: Callable[[U, U], float] = lambda a, b: 1.0,
    is_goal_reached_fnct: Callable[[U, U], bool] = lambda a, b: a == b,
) -> Union[Iterable[U], None]:
    """A non-class version of the path finding algorithm"""

    class FindPath(AStar):
        def heuristic_cost_estimate(self, current: U, goal: U) -> float:
            return heuristic_cost_estimate_fnct(current, goal)  # type: ignore

        def distance_between(self, n1: U, n2: U) -> float:
            return distance_between_fnct(n1, n2)

        def neighbors(self, node) -> Iterable[U]:
            return neighbors_fnct(node)  # type: ignore

        def is_goal_reached(self, current: U, goal: U) -> bool:
            return is_goal_reached_fnct(current, goal)

    return FindPath().astar(start, goal, reversePath)


__all__ = ["AStar", "find_path"]
