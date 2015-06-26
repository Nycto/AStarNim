import astar, unittest, sets, sequtils, strutils, ropes, sets, math

type
    Grid = seq[seq[int]]

    XY = tuple[x, y: int]

proc grid( ascii: varargs[string] ): Grid =
    ## Creates a Grid from ascii art strings
    result = @[]
    for asciiRow in ascii:
        var row: seq[int] = @[]
        for point in asciiRow:
            case point
            of ' ': discard
            of '.': row.add(0)
            of '`': row.add(1)
            of '-': row.add(2)
            of '*': row.add(5)
            of '0'..'9': row.add(parseInt($point))
            else: row.add(-1)
        result.add(row)

proc cost[T]( grid: Grid, a, b: XY ): T =
    ## Returns the cost associated with moving to a point
    return T( grid[b.y][b.x] )

template yieldIfExists( grid: Grid, point: XY ) =
    ## Checks if a point exists within a grid, then yields it if it does
    let isValid =
        point.y >= 0 and point.y < grid.len and
        point.x >= 0 and point.x < grid[point.y].len and
        grid[point.y][point.x] >= 0
    if isValid:
        yield point

iterator neighbors*( grid: Grid, point: XY ): XY =
    ## Yields the connected neighbors of a point
    yieldIfExists( grid, (x: point.x - 1, y: point.y) )
    yieldIfExists( grid, (x: point.x + 1, y: point.y) )
    yieldIfExists( grid, (x: point.x, y: point.y - 1) )
    yieldIfExists( grid, (x: point.x, y: point.y + 1) )

proc `$`( point: XY ): string =
    ## Converts a point to a readable string
    return "(" & $point.x & ", " & $point.y & ")"

proc str( title: string, grid: Grid, path: openArray[XY] ): string =
    ## Converts a grid to a string

    let pathPoints = toSet(path)

    var str = rope(title)
    str.add(":\n")

    for y in countup(0, grid.len - 1):
        for x in countup(0, grid[y].len - 1):
            if pathPoints.contains( (x: x, y: y) ):
                str.add("@")
            elif grid[y][x] < 0:
                str.add("#")
            else:
                case grid[y][x]
                of 0: str.add(".")
                of 1: str.add("`")
                of 2: str.add("-")
                of 5: str.add("*")
                else: str.add($grid[y][x])
        str.add("\n")

    str.add("Path:")
    for row in countup(0, int(floor(path.len / 5))):
        str.add("\n  ")
        for i in countup(row * 5, min(path.len - 1, row * 5 + 4)):
            str.add(`$`(path[i]))
            str.add(" -> ")
    str.add("End\n")

    return $str

template assert(
    within: Grid, starting: XY, to: XY, equals: openArray[XY],
    heuristic: expr, distance: typedesc = float
) =
    ## Asserts a path is created across the given grid
    let route = toSeq(path[Grid, XY, distance](within, starting, to, heuristic))
    checkpoint( str("Expected", within, equals) )
    checkpoint( str("Actual", within, route) )
    assert( route == @equals )

proc walk( start: XY, directions: string ): seq[XY] =
    ## Creates a sequence of points from a string of movements
    result = @[ start ]
    var current = start
    for movement in directions:
        case movement
        of '<': current = (current.x - 1, current.y)
        of '>': current = (current.x + 1, current.y)
        of '^': current = (current.x, current.y - 1)
        of 'v': current = (current.x, current.y + 1)
        else: discard
        if current != result[result.len - 1]:
            result.add(current)

template assert(
    within: Grid, starting: XY, to: XY, equals: string,
    heuristic: expr, distance: typedesc = float
) =
    assert(
        within, starting, to, walk(starting, equals),
        heuristic, distance
    )


suite "A* should":

    test "Yield a single point when goal == start":
        assert(
            grid(". . .",
                 ". . .",
                 ". . ."),
            heuristic = asTheCrowFlies,
            starting = (0, 0), to = (0, 0),
            equals = [(0, 0)] )

    test "Yield two points for connected points":
        assert(
            grid(". . .",
                 ". . .",
                 ". . ."),
            heuristic = asTheCrowFlies,
            starting = (0, 0), to = (1, 0),
            equals = [ (0, 0), (1, 0) ] )

    test "Yield nothing if the goal is unreachable":
        assert(
            grid(". . .",
                 ". . #",
                 ". . ."),
            heuristic = asTheCrowFlies,
            starting = (0, 0), to = (2, 1),
            equals = [] )

        assert(
            grid(". # .",
                 "# # .",
                 ". . ."),
            heuristic = asTheCrowFlies,
            starting = (0, 0), to = (2, 2),
            equals = [] )

    test "Short example":
        assert(
            grid(". * .",
                 ". # .",
                 ". . ."),
            heuristic = asTheCrowFlies,
            starting = (0, 0), to = (2, 2),
            equals = "v v > >")


    let complexGrid = grid(
        ". . . . . . . . . .",
        ". . . . * * . . . .",
        ". . . . * * * . . .",
        ". . . . * * * * . .",
        ". . . * * * * * . .",
        ". . . * * * * * . .",
        ". . . . * * * . . .",
        ". # # # * * * . . .",
        ". # # # * * . . . .",
        ". . . . . . . . . .")

    test "Complex example":
        assert(
            within = complexGrid,
            heuristic = asTheCrowFlies,
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > v > v > v v v" )

    test "Using a manhatten distance":
        assert(
            within = complexGrid,
            heuristic = manhattan,
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > > > v v v v v",
            distance = int)

    test "Using a chebyshev distance":
        assert(
            grid(
                ". . . . . . . ",
                ". . . . * * . ",
                ". . . . * * * ",
                ". . . . 2 2 * ",
                "# # # * 2 2 * ",
                ". . . * * * * ",
                ". . . . . . . "),
            heuristic = chebyshev,
            starting = (1, 1), to = (3, 6),
            equals = "v v > > > v v v <",
            distance = int)

    test "Complex heuristic":

        proc weight (node, start, goal, cameFrom: XY): float {.procvar.}=
            result = asTheCrowFlies(node, goal)
            if cameFrom.x == node.x:
                result = result * 1.5

        assert(
            within = complexGrid,
            heuristic = weight,
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > > > v v v v v" )

    test "onLineToGoal":
        assert(
            within = grid(
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . ."),
            heuristic = onLineToGoal[XY, float](1.5, asTheCrowFlies),
            starting = (1, 1), to = (12, 5),
            equals = "> v > > > v > > > v > > > v >" )

    test "straightLine":
        assert(
            within = complexGrid,
            heuristic = straightLine[XY, float](1.2, manhattan[XY, float]),
            starting = (1, 4), to = (8, 2),
            equals = "^ ^ > > ^ ^ > > > > > v v" )

    test "Usage as an iterator":
        let start: XY = (3, 3)
        let goal: XY = (3, 6)
        var result: seq[XY] = @[]
        for point in path[Grid, XY, int](complexGrid, start, goal, manhattan):
            result.add(point)
        checkpoint( str("Actual", complexGrid, result) )
        assert( result == walk(start, "< v v v >") )

