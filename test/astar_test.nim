import astar, unittest, sets, sequtils, strutils, ropes, sets, options, math, hashes

type
    Grid = seq[seq[int]]
    XY = tuple[x, y: int]

template defineGrid( name: typedesc, body: stmt ) {.immediate.} =
    ## Creates a distinct phantom grid type for using a custom heuristic
    type name = distinct Grid
    converter toGrid(g: name): Grid = Grid(g)
    proc `[]`( grid: name, index: int ): seq[int] = Grid(grid)[index]
    body

defineGrid(CrowGrid):
    ## A grid that uses 'asTheCrowFlies' for the heuristic
    proc heuristic( grid: CrowGrid, a, b: XY ): float = asTheCrowFlies(a, b)

defineGrid(ManhattanGrid):
    ## A grid that uses 'manhattan' for the heuristic
    proc heuristic( grid: ManhattanGrid, a, b: XY ): int =
        manhattan[XY, int](a, b)

defineGrid(ChebyshevGrid):
    ## A grid that uses 'chebyshev' for the heuristic
    proc heuristic( grid: ChebyshevGrid, a, b: XY ): int =
        chebyshev[XY, int](a, b)

defineGrid(LineToGoalGrid):
    ## A grid that prioritizes nodes on the line to the goal
    proc heuristic(
        grid: LineToGoalGrid, node, start, goal, cameFrom: XY
    ): float =
        return 1.5 * onLineToGoal[XY, float](node, start, goal) +
            asTheCrowFlies(node, goal)

defineGrid(StraightLineGrid):
    ## A grid that prioritizes paths that don't have any turns
    proc heuristic(
        grid: StraightLineGrid,
        node, start, goal, cameFrom: XY,
        grandparent: Option[XY]
    ): float =
        straightLine[XY, float](1.2, node, grandparent) *
            manhattan[XY, float](node, goal)

type AnyGrid = ## A unified type for all known test grids
    CrowGrid|ManhattanGrid|ChebyshevGrid|LineToGoalGrid|StraightLineGrid

proc cost( grid: AnyGrid, a, b: XY ): int =
    ## Returns the cost associated with moving to a point
    return int( grid[b.y][b.x] )

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

template yieldIfExists( grid: AnyGrid, point: XY ) =
    ## Checks if a point exists within a grid, then yields it if it does
    let isValid =
        point.y >= 0 and point.y < grid.len and
        point.x >= 0 and point.x < grid[point.y].len and
        grid[point.y][point.x] >= 0
    if isValid:
        yield point

iterator neighbors*( grid: AnyGrid, point: XY ): XY =
    ## Yields the connected neighbors of a point
    yieldIfExists( grid, (x: point.x - 1, y: point.y) )
    yieldIfExists( grid, (x: point.x + 1, y: point.y) )
    yieldIfExists( grid, (x: point.x, y: point.y - 1) )
    yieldIfExists( grid, (x: point.x, y: point.y + 1) )

proc `$`( point: XY ): string =
    ## Converts a point to a readable string
    return "(" & $point.x & ", " & $point.y & ")"

proc str( title: string, grid: Grid|AnyGrid, path: openArray[XY] ): string =
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
    gridType: typedesc, within: Grid, starting: XY, to: XY,
    equals: openArray[XY], distance: typedesc
) =
    ## Asserts a path is created across the given grid
    let route = toSeq(
        path[gridType, XY, distance](gridType(within), starting, to))
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
    gridType: typedesc,
    within: Grid, starting: XY, to: XY, equals: string,
    distance: typedesc
) =
    assert(gridType, within, starting, to, walk(starting, equals), distance)


suite "A* should":

    test "Yield a single point when goal == start":
        assert(
            gridType = CrowGrid,
            grid(". . .",
                 ". . .",
                 ". . ."),
            starting = (0, 0), to = (0, 0),
            equals = [(0, 0)],
            distance = float )

    test "Yield two points for connected points":
        assert(
            gridType = CrowGrid,
            grid(". . .",
                 ". . .",
                 ". . ."),
            starting = (0, 0), to = (1, 0),
            equals = [ (0, 0), (1, 0) ],
            distance = float )

    test "Yield nothing if the goal is unreachable":
        assert(
            gridType = CrowGrid,
            grid(". . .",
                 ". . #",
                 ". . ."),
            starting = (0, 0), to = (2, 1),
            equals = [],
            distance = float )

        assert(
            gridType = CrowGrid,
            grid(". # .",
                 "# # .",
                 ". . ."),
            starting = (0, 0), to = (2, 2),
            equals = [],
            distance = float )

    test "Short example":
        assert(
            gridType = CrowGrid,
            grid(". * .",
                 ". # .",
                 ". . ."),
            starting = (0, 0), to = (2, 2),
            equals = "v v > >",
            distance = float )


    let complexGrid: Grid = grid(
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
            gridType = CrowGrid,
            within = complexGrid,
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > v > v > v v v",
            distance = float )

    test "Using a manhattan distance":
        assert(
            gridType = ManhattanGrid,
            within = complexGrid,
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > > > v v v v v",
            distance = int)

    test "Using a chebyshev distance":
        assert(
            gridType = ChebyshevGrid,
            grid(
                ". . . . . . . ",
                ". . . . * * . ",
                ". . . . * * * ",
                ". . . . 2 2 * ",
                "# # # * 2 2 * ",
                ". . . * * * * ",
                ". . . . . . . "),
            starting = (1, 1), to = (3, 6),
            equals = "v v > > > v v v <",
            distance = int)

    test "onLineToGoal":
        assert(
            gridType = LineToGoalGrid,
            within = grid(
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . .",
                ". . . . . . . . . . . . . ."),
            starting = (1, 1), to = (12, 5),
            equals = "> v > > > v > > > v > > > v >",
            distance = float )

    test "straightLine":
        assert(
            gridType = StraightLineGrid,
            within = complexGrid,
            starting = (1, 4), to = (8, 2),
            equals = "^ ^ > > ^ ^ > > > > > v v",
            distance = float )

    test "Usage as an iterator":
        let start: XY = (3, 3)
        let goal: XY = (3, 6)
        var result: seq[XY] = @[]

        for point in path[CrowGrid, XY, int](
                CrowGrid(complexGrid), start, goal):
            result.add(point)

        checkpoint( str("Actual", complexGrid, result) )
        assert( result == walk(start, "< v v v >") )

