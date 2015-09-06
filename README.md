AStarNim [![Build Status](https://travis-ci.org/Nycto/AStarNim.svg?branch=master)](https://travis-ci.org/Nycto/AStarNim)
========

The classic A-Star path finding algorith built in Nim

For more information about A-Star itself, the folks over at Red Blob Games put
together a very comprehensive introduction:

http://www.redblobgames.com/pathfinding/a-star/introduction.html

Usage
-----

This is a "Bring Your Own Graph" implementation. Meaning, this implementation
relies on your own graph implementation. This library will then call your
functions and pluck out the edges of your graph.

This was done to avoid needless data copying. In most cases, your data will
already be represented in a data structure somewhere. Having to keep two
parallel copies in place doesn't make sense in this case.

So, to actually hook your graph into this algorithm, there are three functions
you need:

* `neighbors`: An iterator that returns the neighbors of a node
* `cost`: A proc that returns the cost of moving from one node to another
* `heuristic`: A proc that determines the priority of a node. This is the real
  magic of A-Star; the heuristic determines which nodes are looked at first so
  it can skip looking at obviously wrong nodes. There are a few implementations
  provided by default. For example, `asTheCrowFlies` and `manhattan`. See the
  API docs for more information.

Example code is provided below. What you'll notice is that the code required to
get a grid up and running takes longer than actually using the algorithm. As
mentioned before, this is on purpose. Odds are that you will have already done
this work for some other part of your application, so this disappears in the
real world.

API Docs
--------

http://nycto.github.io/AStarNim/astar.html

Defining a Graph
----------------

To connect a graph with the AStar library, you must implement three functions:

1. '''neighbors''', which iterates over the neighbors of a node. Its
   signature should look like this:

   ```nimrod
   iterator neighbors*(grid: MyGraph, node: MyNode): Node
   ```

2. '''cost''', which returns the exact price for moving between nodes. It
   should look like this:

   ```nimrod
   proc cost*(grid: MyGraph, a, b: MyNode): Distance
   ```

3. '''heuristic''', which estimates the how close a point is to the goal. It
   has three possible signatures:

   ```
   proc heuristic*(grid: MyGraph, next, goal: MyNode): Distance
   ```

   ```
   proc heuristic*(grid: MyGraph, next, start, goal, parent: MyNode): Distance
   ```

   ```
   proc heuristic*(
       grid: MyGraph, next, start, goal, parent: MyNode,
       grandparent: Option[MyNode]
   ): Distance
   ```

Full Example
------------

```nimrod
import astar

type
    Grid = seq[seq[int]]
        ## A matrix of nodes. Each cell is the cost of moving to that node

    Point = tuple[x, y: int]
        ## A point within that grid

template yieldIfExists( grid: Grid, point: Point ) =
    ## Checks if a point exists within a grid, then calls yield it if it does
    let exists =
        point.y >= 0 and point.y < grid.len and
        point.x >= 0 and point.x < grid[point.y].len
    if exists:
        yield point

iterator neighbors*( grid: Grid, point: Point ): Point =
    ## An iterator that yields the neighbors of a given point
    yieldIfExists( grid, (x: point.x - 1, y: point.y) )
    yieldIfExists( grid, (x: point.x + 1, y: point.y) )
    yieldIfExists( grid, (x: point.x, y: point.y - 1) )
    yieldIfExists( grid, (x: point.x, y: point.y + 1) )

proc cost*(grid: Grid, a, b: Point): float =
    ## Returns the cost of moving from point `a` to point `b`
    float(grid[a.y][a.x])

proc heuristic*( grid: Grid, node, goal: Point ): float =
    ## Returns the priority of inspecting the given node
    asTheCrowFlies(node, goal)

# A sample grid. Each number represents the cost of moving to that space
let grid = @[
    @[ 0, 0, 0, 0, 0 ],
    @[ 0, 3, 3, 3, 0 ],
    @[ 0, 3, 5, 3, 0 ],
    @[ 0, 3, 3, 3, 0 ],
    @[ 0, 0, 0, 0, 0 ]
]

let start: Point = (x: 0, y: 3)
let goal: Point = (x: 4, y: 3)

# Pass in the start and end points and iterate over the results.
for point in path[Grid, Point, float](grid, start, goal):
    echo point
```

License
-------

This library is released under the MIT License, which is pretty spiffy. You
should have received a copy of the MIT License along with this program. If
not, see http://www.opensource.org/licenses/mit-license.php



