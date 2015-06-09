##
## Classic A-Star path finding
##
## For more information about A-Star itself, the folks over at Red Blob Games
## put together a very comprehensive introduction:
##
## http://www.redblobgames.com/pathfinding/a-star/introduction.html
##

import binaryheap, tables, hashes, math

type
    Distance* = int|float
        ## Distance is used for two things:
        ## 1. To measure cost of moving between nodes
        ## 2. To represent the heuristic that determines how close a point
        ##    is to the goal

    Node* = concept n
        ## Represents a node stored within a graph.
        ## * `==`: Nodes must be comparable so we know when we reach the goal
        ## * `hash`: Nodes are used as keys in a table, so they need to be
        ##   hashable
        `==`(n, n) is bool
        `hash`(n) is THash

    Graph* = concept g
        ## The graph being traversed.
        ## * `nieghbors`: Iterates over the neighbors of a node in the graph
        neighbors(g, Node) is iterator Node

    AStar* [G, N, D] = object
        ## The configured A* interface.
        ## * `[G]`: The type for the overall graph
        ## * `[N]`: The type of element that represents nodes in the graph
        ## * `[D]`: The numeric type to use for costs and the heuristic. See
        ##   `Distance` above.
        ## * `graph`: is a reference to the graph being traversed
        ## * `cost`: Returns the cost of moving from one node to another
        ## * `heuristic`: Estimates the distance between two nodes in the graph
        graph: G
        cost: proc ( grid: G, a, b: N ): D
        heuristic: proc (a, b: N): D

    Point* = concept p
        ## An X/Y Coordinate. This isn't used by the A-Star algorithm itself,
        ## but by the built in heuristic procs.
        p.x is Distance
        p.y is Distance


proc asTheCrowFlies*( a, b: Point ): float {.procvar.} =
    ## A convenience function that measures the exact distance between two
    ## points. This is meant to be used as the heuristic when creating a new
    ## `AStar` instance.
    return sqrt(
        pow(float(a.x) - float(b.x), 2) +
        pow(float(a.y) - float(b.y), 2) )

proc manhattan*(a, b: Point): auto {.procvar.} =
    ## A convenience function that measures the manhattan distance between two
    ## points. This is meant to be used as the heuristic when creating a new
    ## `AStar` instance.
    return abs(a.x - b.x) + abs(a.y - b.y)

proc chebyshev*(a, b: Point): auto {.procvar.} =
    ## A convenience function that measures the chebyshev distance between two
    ## points. This is also known as the diagonal distance. This is meant to be
    ## used as the heuristic when creating a new `AStar` instance.
    return max(abs(a.x - b.x), abs(a.y - b.y))


proc newAStar*[G: Graph, N: Node, D: Distance](
    graph: G,
    heuristic: proc(a, b: N): D,
    cost: proc ( grid: G, a, b: N ): D
): AStar[G, N, D] =
    ## Creates a new AStar instance. See the `AStar` object above for a
    ## description of types and parameters
    result = AStar[G, N, D]( graph: graph, heuristic: heuristic, cost: cost )


type
    FrontierElem[N, D] = tuple[node: N, priority: D]
        ## Internally used to associate a graph node with how much it costs

    CameFrom[N, D] = tuple[node: N, cost: D]
        ## Given a node, this stores the node you need to backtrack to to get
        ## to this node and how much it costs to get here


iterator backtrack[N, D](
    cameFrom: Table[N, CameFrom[N, D]], start, goal: N
): N =
    ## Once the table of back-references is filled, this yields the reversed
    ## path back to the consumer
    yield start

    var current: N = goal
    var path: seq[N] = @[]

    while current != start:
        path.add(current)
        current = `[]`(cameFrom, current).node

    for i in countdown(path.len - 1, 0):
        yield path[i]

iterator path*[G: Graph, N: Node, D: Distance](
    astar: AStar[G, N, D], start, goal: N
): N =
    ## Executes the A-Star algorithm and iterates over the nodes that connect
    ## the start and goal

    # The frontier is the list of nodes we need to visit, sorted by a
    # combination of cost and how far we estimate them to be from the goal
    var frontier =
        newHeap[FrontierElem[N, D]] do (a, b: FrontierElem[N, D]) -> int:
            return cmp(a.priority, b.priority)

    # Put the start node into the frontier so we have a place to kick off
    frontier.push( (node: start, priority: D(0)) )

    # A map of backreferences. After getting to the goal, you use this to walk
    # backwards through the path and ultimately find the reverse path
    var cameFrom = initTable[N, CameFrom[N, D]]()

    while frontier.size > 0:
        let current = frontier.pop

        # Now that we have a map of back-references, yield the path back out to
        # the caller
        if current.node == goal:
            for node in backtrack(cameFrom, start, goal):
                yield node
            break

        let currentCost = `[]`(cameFrom, current.node).cost

        for next in astar.graph.neighbors(current.node):

            # The intrinsic cost of moving into this node
            let nodeCost = astar.cost(astar.graph, current.node, next)

            # Adding current cost lets us track the total it took to get here
            let cost = currentCost + nodeCost

            # If we haven't seen this point already, or we found a cheaper
            # way to get to that
            if not cameFrom.hasKey(next) or cost < `[]`(cameFrom, next).cost:

                # Add this node to the backtrack map
                `[]=`(cameFrom, next, (node: current.node, cost: cost))

                # Also add it to the frontier so we check out its neighbors
                frontier.push((
                    node: next,
                    priority: cost + astar.heuristic(next, goal)
                ))

