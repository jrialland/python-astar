python-astar
============

This is a simple implementation of the [a-star path finding algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) in python

## Documentation

The astar module defines the AStar class, which has to be inherited from and completed with the implementation of several methods :

## neighbors
```py
@abstractmethod
def neighbors(self, node)

```

For a given node, returns (or yields) the list of its neighbors. this method must be implemented in a subclass

This is the method that one would provide in order to give to the algorithm the description of the graph to use during for computation

This method must be implemented in a subclass.

## distance\_between
```py
@abstractmethod
def distance_between(self, n1, n2)

```

Gives the real distance/cost between two adjacent nodes n1 and n2 (i.e n2 belongs to the list of n1's neighbors).
n2 is guaranteed to belong to the list returned by a call to neighbors(n1).

This method must be implemented in a subclass.


## heuristic\_cost\_estimate
```py
@abstractmethod
def heuristic_cost_estimate(self, current, goal)

```

Computes the estimated (rough) distance/cost between a node and the goal. The first argument is the start node, or any node that have been returned by a call to the neighbors() method.

This method is used to give to the algorithm an hint about the node he may try next during search.

This method must be implemented in a subclass.


## is\_goal\_reached
```py

def is_goal_reached(self, current, goal)

```

This method shall return a truthy value when the goal is 'reached'. By default it checks that current == goal.


### Example

The maze solver example is executed by running

``PYTHONPATH=./src python ./src/test/maze.py``

```
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
```
