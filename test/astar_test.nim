import astar, unittest, sets, sequtils, strutils, ropes, sets, math

type
    Grid = object
        rows: seq[seq[int]]

    XY = tuple[x, y: int]

proc grid( ascii: varargs[string] ): Grid =
    ## Creates a Grid from ascii art strings
    var rows: seq[seq[int]] = @[]
    for asciiRow in ascii:
        var row: seq[int] = @[]
        for point in asciiRow:
            case point
            of ' ': discard
            of '.': row.add(0)
            of '*': row.add(5)
            of '0'..'9': row.add(parseInt($point))
            else: row.add(-1)
        rows.add(row)
    return Grid(rows: rows)

proc cost( grid: Grid, a, b: XY ): float =
    ## Returns the cost associated with moving to a point
    return float(grid.rows[b.y][b.x])

iterator neighbors*( grid: Grid, point: XY ): XY =
    ## Yields the connected neighbors of a point
    let adjacent = [
        (x: point.x - 1, y: point.y),
        (x: point.x + 1, y: point.y),
        (x: point.x, y: point.y - 1),
        (x: point.x, y: point.y + 1)
    ]
    for adj in adjacent:
        if adj.y >= 0 and adj.y < grid.rows.len:
            if adj.x >= 0 and adj.x < grid.rows[adj.y].len:
                if grid.rows[adj.y][adj.x] >= 0:
                    yield adj

proc `$`( point: XY ): string =
    ## Converts a point to a readable string
    return "(" & $point.x & ", " & $point.y & ")"

proc str( title: string, grid: Grid, path: openArray[XY] ): string =
    ## Converts a grid to a string

    let pathPoints = toSet(path)

    var str = rope(title)
    str.add(":\n")

    for y in countup(0, grid.rows.len - 1):
        for x in countup(0, grid.rows.len - 1):
            if pathPoints.contains( (x: x, y: y) ):
                str.add("@")
            elif grid.rows[y][x] < 0:
                str.add("#")
            elif grid.rows[y][x] == 5:
                str.add("*")
            elif grid.rows[y][x] == 0:
                str.add(".")
            else:
                str.add($grid.rows[y][x])
            str.add(" ")
        str.add("\n")

    str.add("Path:")
    for row in countup(0, int(floor(path.len / 5))):
        str.add("\n  ")
        for i in countup(row * 5, min(path.len - 1, row * 5 + 4)):
            str.add(`$`(path[i]))
            str.add(" -> ")
    str.add("End\n")

    return $str

proc assert( within: Grid, starting: XY, to: XY, equals: openArray[XY] ) =
    ## Asserts a path is created across the given grid
    let astar = newAStar[Grid, XY, float](within, asTheCrowFlies)
    let path = toSeq( path[Grid, XY, float](astar, starting, to) )
    checkpoint( str("Actual", within, path) )
    checkpoint( str("Expected", within, equals) )
    assert( path == @equals )

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

proc assert( within: Grid, starting: XY, to: XY, equals: string ) =
    assert( within, starting, to, walk(starting, equals) )


suite "A* should":

    test "Yield a single point when goal == start":
        assert(
            grid(". . .",
                 ". . .",
                 ". . ."),
            starting = (0, 0), to = (0, 0),
            equals = [(0, 0)] )

    test "Yield two points for connected points":
        assert(
            grid(". . .",
                 ". . .",
                 ". . ."),
            starting = (0, 0), to = (1, 0),
            equals = [ (0, 0), (1, 0) ] )

    test "Yield nothing if the goal is unreachable":
        assert(
            grid(". . .",
                 ". . #",
                 ". . ."),
            starting = (0, 0), to = (2, 1),
            equals = [] )

        assert(
            grid(". # .",
                 "# # .",
                 ". . ."),
            starting = (0, 0), to = (2, 2),
            equals = [] )

    test "Short example":
        assert(
            grid(". * .",
                 ". # .",
                 ". . ."),
            starting = (0, 0), to = (2, 2),
            equals = "v v > >")

    test "Complex example":
        assert(
            grid(". . . . . . . . . .",
                 ". . . . * * . . . .",
                 ". . . . * * * . . .",
                 ". . . . * * * * . .",
                 ". . . * * * * * . .",
                 ". . . * * * * * . .",
                 ". . . . * * * . . .",
                 ". # # # * * * . . .",
                 ". # # # * * . . . .",
                 ". . . . . . . . . ."),
            starting = (1, 4), to = (8, 5),
            equals = "> ^ > ^ ^ ^ > > > v > v > v v v" )


