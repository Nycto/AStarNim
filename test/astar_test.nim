import astar, unittest, sets, sequtils, strutils, ropes

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
            of '0'..'9': row.add(parseInt($point))
            else: row.add(-1)
        rows.add(row)
    return Grid(rows: rows)

proc cost( grid: Grid, a, b: XY ): int =
    ## Returns the cost associated with moving to a point
    return grid.rows[b.y][b.x]

proc isValid( grid: Grid, point: XY ): bool =
    ## Returns whether a point exists in this grid

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

proc `$`( grid: Grid ): string =
    ## Converts a grid to a string
    var str = rope("")
    for row in grid.rows:
        if str.len != 0:
            str.add("\n")
        for column in row:
            if column < 0:
                str.add("#")
            elif column == 0:
                str.add(".")
            else:
                str.add($column)
            str.add(" ")
    return $str

proc `$`( point: XY ): string =
    ## Converts a point to a readable string
    return "(" & $point.x & ", " & $point.y & ")"

proc createAStar( grid: Grid ): AStar[Grid, XY] =
    ## Creates an AStar instance prefilled to use a manhatten distance heuristic
    return newAStar[Grid, XY](grid) do (a, b: XY) -> int:
        return abs(a.x - b.x) + abs(a.y - b.y)

proc assert( within: Grid, starting: XY, to: XY, equals: openArray[XY] ) =
    ## Asserts a path is created across the given grid
    let astar = createAStar(within)
    let path = toSeq( path[Grid, XY](astar, starting, to) )
    checkpoint("Grid is:\n" & $within)
    checkpoint("Expected: " & join(map(equals, `$`), " -> "))
    checkpoint("Actual:   " & join(map(path, `$`), " -> "))
    assert( path == @equals )


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

