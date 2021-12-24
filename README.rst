.. image:: https://badge.fury.io/py/astar.svg
    :target: https://badge.fury.io/py/astar

.. image:: https://travis-ci.com/jrialland/python-astar.svg?branch=master
    :target: https://travis-ci.com/jrialland/python-astar

.. image:: https://coveralls.io/repos/github/jrialland/python-astar/badge.svg?branch=master
    :target: https://coveralls.io/github/jrialland/python-astar?branch=master

.. image:: https://img.shields.io/github/stars/jrialland/python-astar
    :target: https://github.com/jrialland/python-astar

.. image:: https://img.shields.io/github/forks/jrialland/python-astar
    :target: https://github.com/jrialland/python-astar

python-astar
============

This is a simple implementation of the `a-star path finding
algorithm <https://en.wikipedia.org/wiki/A*_search_algorithm>`__ in
python

Documentation
-------------

The `astar` module defines the `AStar` class, which has to be inherited from
and completed with the implementation of several methods.

The functions take/return _node_ objects.
The `astar` library only requires the following property from these objects:

- They must be hashable (i.e. implement `__hash__`).

For the default implementation of `is_goal_reached`, the objects must be
comparable for same-ness (i.e. implement `__eq__`).

A simple way to achieve this, is to use simple objects based on strings,
floats, integers, tuples.
[`dataclass`](https://docs.python.org/3/library/dataclasses.html#dataclasses.dataclass)
objects declared with `@dataclass(frozen=True)` directly implement `__hash__`
if possible.


neighbors
~~~~~~~~~

.. code:: py

    @abstractmethod
    def neighbors(self, node)

For a given node, returns (or yields) the list of its neighbors.

This is the method that one would provide in order to give to the
algorithm the description of the graph to use during for computation.

This method must be implemented in a subclass.

distance\_between
~~~~~~~~~~~~~~~~~

.. code:: py

    @abstractmethod
    def distance_between(self, n1, n2)

Gives the real distance/cost between two adjacent nodes n1 and n2 (i.e
n2 belongs to the list of n1's neighbors). n2 is guaranteed to belong to
the list returned by a call to neighbors(n1).

This method must be implemented in a subclass.

heuristic\_cost\_estimate
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: py

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal)

Computes the estimated (rough) distance/cost between a node and the
goal. The first argument is the start node, or any node that have been
returned by a call to the neighbors() method.

This method is used to give to the algorithm an hint about the node he
may try next during search.

This method must be implemented in a subclass.

is\_goal\_reached
~~~~~~~~~~~~~~~~~

.. code:: py


    def is_goal_reached(self, current, goal)

This method shall return a truthy value when the goal is 'reached'. By
default it checks that `current == goal`.


"Functional" API.
~~~~~~~~~~~~~~~~~

If you dislike to have to inherit from the AStar class and create an instance in order to run the algorithm, the module also provides a "find_path" function, which takes functions as parameters and provides reasonnable defaults for some of them.

See <https://github.com/jrialland/python-astar/blob/master/tests/basic/test_basic.py>

.. code:: py

    def find_path(
    	start,
    	goal,
    	neighbors_fnct,
    	reversePath=False,
    	heuristic_cost_estimate_fnct = lambda a, b: Infinite,
    	distance_between_fnct = lambda a, b: 1.0,
    	is_goal_reached_fnct = lambda a, b: a == b
    	)

Examples
--------

Maze solver
~~~~~~~~~~~

This script generates an ascii maze, and finds the path between the upper left corner and the bottom right

``PYTHONPATH=. python tests/maze/test_maze.py``

::

    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    |####    |     |              |        |              |     |
    +--+# +  +  +  +  +--+--+--+  +  +--+  +--+--+--+  +--+  +  +
    | ### |  |  |  |  |        |  |     |     |        |     |  |
    + #+--+--+  +  +  +--+  +--+  +  +--+--+  +  +--+--+  +--+  +
    | #|        |  |  |     |     |  |        |  |     |  |     |
    + #+  +--+--+  +  +  +--+  +--+  +  +--+--+  +  +  +  +  +--+
    | #|        |  |  |     |     |  |     |        |     |     |
    + #+--+--+  +  +  +--+  +--+  +  +--+--+  +--+--+--+--+--+  +
    | #      |  |  |  |        |     | ### |  |     |        |  |
    + #+--+--+  +  +  +  +--+--+--+--+ #+# +  +--+  +  +--+  +  +
    | #         |     |       ####| ####|# |  |     |     |  |  |
    + #+--+--+--+--+--+--+--+ #+ #+ #+--+# +  +  +  +--+  +  +  +
    | #|    ####|       #######| ####| ### |     |     |  |     |
    + #+--+ #+ #+--+--+ #+--+--+--+--+ #+--+  +--+--+--+  +--+--+
    | ####| #| ##########|           | ### |  | ###### |        |
    +--+ #+ #+--+--+--+--+  +--+--+  +--+# +--+ #+--+# +--+--+  +
    |  | ####|        |     |           |########|  |##| ### |  |
    +  +--+--+  +--+  +  +--+  +--+--+  +--+--+--+  + #+ #+# +  +
    |        |     |  |  |     |                    | ####|#### |
    +  +--+--+--+  +  +  +  +--+  +--+--+--+--+--+  +--+--+--+# +
    |  |           |     |     |     | ####|     |     | ###### |
    +  +  +--+--+--+--+--+  +  +--+--+##+ #+--+  +--+  + #+--+--+
    |     |  |           |  |  | ###### | ####|        | ### |  |
    +  +--+  +  +--+--+  +--+  + #+--+--+--+ #+--+--+--+--+# +  +
    |        |  |     |        | ###### |  | ############ |# |  |
    +--+--+--+  +  +  +--+--+  +--+--+# +  +--+--+--+--+# +# +  +
    |           |  |  |        | ###### | ##########|  |#### |  |
    +  +--+  +--+--+  +  +--+--+ #+--+--+ #+--+--+ #+  +--+--+  +
    |  |     |     |        | ####|     | #######| ############ |
    +  +--+--+  +  +--+  +--+ #+--+--+  +  +--+ #+--+--+--+--+# +
    |        |  |     |  | ####| ####|        | #| ### |     |##|
    +--+--+  +  +--+  +  + #+--+ #+ #+--+--+  + #+ #+# +  +  + #+
    |        |  |     |  | #######| ####|     | #| #|# |  |  | #|
    +  +--+  +  +  +--+--+--+--+--+--+ #+--+--+ #+ #+# +--+  + #+
    |  |     |  |  |                 | #| ####| ####|# |     | #|
    +  +  +--+  +  +  +--+--+--+--+  + #+ #+ #+--+--+# +  +  + #+
    |  |  |     |  |        |     |  | ####| ######### |  |  | #|
    +  +--+  +--+  +--+--+  +  +  +  +--+--+--+--+--+--+  +--+ #+
    |           |              |  |                            #|
    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    
   
London Underground
~~~~~~~~~~~~~~~~~~

This script finds the shortest path between two underground stations, based on a list of London's stations

``PYTHONPATH=. python tests/london/test_london_underground.py Chesham Beckton``

::

    Chesham
    Chalfont & Latimer
    Chorleywood
    Rickmansworth
    Moor Park
    Northwood
    Northwood Hills
    Pinner
    North Harrow
    Harrow-on-the-Hill
    Northwick Park
    Preston Road
    Wembley Park
    Finchley Road
    Baker Street
    Bond Street
    Oxford Circus
    Tottenham Court Road
    Holborn
    Chancery Lane
    St. Paul's
    Bank
    Shadwell
    Limehouse
    Westferry
    Poplar
    Blackwall
    East India
    Canning Town
    Royal Victoria
    Custom House
    Prince Regent
    Royal Albert
    Beckton Park
    Cyprus
    Gallions Reach
    Beckton


TAN Network
~~~~~~~~~~~

A solution for a codingame's puzzle (https://www.codingame.com/training/hard/tan-network)

``PYTHONPATH=. python tests/tan_network/test_tan_network_5.py``

.. code:: sh

    .
    ----------------------------------------------------------------------
    Ran 1 test in 0.010s

    OK


