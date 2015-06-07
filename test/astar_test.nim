import astar, unittest, sets, sequtils

type
    Grid = object
        rows: seq[seq[int]]

    XY = tuple[x, y: int]

proc grid( rows: varargs[string] ): Grid =
    discard

proc cost( grid: Grid, a, b: XY ): int = 0

iterator neighbors( grid: Grid, point: XY ): XY =
    discard


suite "A* should":

    const square = grid(
        ".....",
        ".....",
        ".....",
        ".....",
        "....."
    )

    test "Yield nothing when goal == start":
        let astar = newAStar[Grid, XY](square) do (a, b: XY) -> int:
            return abs(a.x - b.x) + abs(a.y - b.y)

        let path = toSeq( path[Grid, XY](astar, (0, 0), (0, 0)) )
        require( path == @[] )

