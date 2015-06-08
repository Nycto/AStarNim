AStarNim [![Build Status](https://travis-ci.org/Nycto/AStarNim.svg?branch=master)](https://travis-ci.org/Nycto/AStarNim)
========

The classic A-Star path finding algorith built in Nim

For more information about A-Star itself, the folks over at Red Blob Games put
together a very comprehensive introduction:

http://www.redblobgames.com/pathfinding/a-star/introduction.html

Usage
-----

This is a "Bring Your Own Graph" implementation. Meaning, this implementation
relies on you to have your data already organized in a graph. This library will
then call your functions and pluck out the edges of your graph.

This was done to avoid copying. In most cases, the data is already represented
in a data structure somewhere. Having to copy it over to yet another location
just doesn't make any sense.

So, to actually provide a graph there are three functions you need:

* `neighbors`: An iterator that returns the neighbors of a node
* `cost`: A proc that returns the cost of moving to a function
* `heuristic`: A function that determines the priority of a node. This is the
  real magic of A-Star; the heuristic determines which nodes are looked at first
  so it can skip looking at obviously wrong nodes. There are two implementations
  provided by default: `asTheCrowFlies` and `manhattan`

Full Example
------------

```nimrod
import astar

type
    # A grid of nodes. Each number represents the cost of movign to that node
    Grid = seq[seq[int]]

    # A point within that grid
    Point = tuple[x, y: int]

# A sample grid
let grid = @[
    @[ 0, 0, 0, 0, 0 ],
    @[ 0, 3, 3, 3, 0 ],
    @[ 0, 3, 5, 3, 0 ],
    @[ 0, 3, 3, 3, 0 ],
    @[ 0, 0, 0, 0, 0 ]
]

# Checks if a point exists within that grid
proc exists( grid: Grid, point: Point ): bool =
    return point.y >= 0 and point.y < grid.len and
        point.x >= 0 and point.x < grid[point.y].len

# An iterator that yields the edges of a given point
iterator neighbors*( grid: Grid, point: Point ): Point =
    let adjacent = [
        (x: point.x - 1, y: point.y),
        (x: point.x + 1, y: point.y),
        (x: point.x, y: point.y - 1),
        (x: point.x, y: point.y + 1)
    ]
    for adj in adjacent:
        if exists(grid, adj):
            yield adj

let pathfinder = newAStar[Grid, Point, int](
    grid, manhattan,
    proc (grid: Grid, a, b: Point): int =
        return grid[a.y][a.x]
)

for point in path[Grid, Point, int]( pathfinder, (x: 0, y: 3), (x: 4, y: 3) ):
    echo point
```

License
-------

This library is released under the MIT License, which is pretty spiffy. You
should have received a copy of the MIT License along with this program. If
not, see http://www.opensource.org/licenses/mit-license.php



