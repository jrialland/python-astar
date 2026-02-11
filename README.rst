.. image:: https://badge.fury.io/py/astar.svg
    :target: https://badge.fury.io/py/astar

.. image:: https://github.com/jrialland/python-astar/actions/workflows/pythonpackage.yml/badge.svg
    :target: https://github.com/jrialland/python-astar/actions/workflows/pythonpackage.yml
    
.. image:: https://coveralls.io/repos/github/jrialland/python-astar/badge.svg?branch=master
    :target: https://coveralls.io/github/jrialland/python-astar?branch=master

.. image:: https://img.shields.io/github/stars/jrialland/python-astar
    :target: https://github.com/jrialland/python-astar

.. image:: https://img.shields.io/github/forks/jrialland/python-astar
    :target: https://github.com/jrialland/python-astar

.. image:: https://repobeats.axiom.co/api/embed/2e7245f5b553bf4fec1eea2e9ba9040b8e7992c7.svg

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

The computation of the hash may be implemented by several means :
 - base types like strings, floats, integers, tuples. already implement __hash__
 - `dataclass https://docs.python.org/3/library/dataclasses.html#dataclasses.dataclass ` objects declared with `@dataclass(frozen=True)` directly implement `__hash__` if possible.
 - by overriding the `__hash__() https://docs.python.org/3/reference/datamodel.html#object.__hash__` method as a combination of the fields that you consider relevant for your object.

neighbors
~~~~~~~~~

.. code:: py

    @abstractmethod
    def neighbors(self, node)

For a given node, returns (or yields) the list of its neighbors.

This is the method that one would provide in order to give to the
algorithm the description of the graph to use during for computation.

Alternately, your override method may be named "path\_neighbors". Instead of
your node, this method receives a "SearchNode" object whose "came_from"
attribute points to the previous node; your node is in its "data" attribute.
You might want to use this if your path is directional, like the track of a
train that can't do 90° turns.

One of these methods must be implemented in a subclass.


distance\_between
~~~~~~~~~~~~~~~~~

.. code:: py

    @abstractmethod
    def distance_between(self, n1, n2)

Gives the real distance/cost between two adjacent nodes n1 and n2 (i.e
n2 belongs to the list of n1's neighbors). n2 is guaranteed to belong to
the list returned by a call to neighbors(n1).

Alternately, you may override "path\_distance\_between". The arguments
will be a "SearchNode", as in "path\_neighbors". You might want to use this
if your distance measure should include the path's attainable speed, the
kind and number of turns on it, or similar. You can use the nodes' "cache"
attributes to store some data, to speed up calculation.

One of these methods must be implemented in a subclass.


heuristic\_cost\_estimate
~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: py

    @abstractmethod
    def heuristic_cost_estimate(self, current, goal)

Computes the estimated (rough) distance/cost between a node and the
goal. The first argument is the start node, or any node that have been
returned by a call to the neighbors() method.

This method is used to give to the algorithm an hint about the node it
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


.. image:: https://api.star-history.com/svg?repos=jrialland/python-astar&type=date&legend=top-left
    :target: https://www.star-history.com/#jrialland/python-astar&type=date&legend=top-left
