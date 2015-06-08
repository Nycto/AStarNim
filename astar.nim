##
## A-Star path finding
## @see http://www.redblobgames.com/pathfinding/a-star/introduction.html
##

import binaryheap, tables, hashes, math

type
    Distance* = int|float ## \
        ## Distance is used to measure cost and the heuristic between two points

    Node* = concept n ## \
        ## Represents a node stored within a graph

        `==`(n, n) is bool ## \
            ## Nodes must be comparable so we know when we reach the goal

        `hash`(n) is THash ## \
            ## Nodes are used as keys in a table, so they need to be hashable

    Graph* = concept g ## \
        ## The graph being traversed

        neighbors(g, Node) is iterator Node ## \
            ## Allows iteration over the neighbors of a node in the graph

        cost(g, Node, Node) is Distance ## \
            ## Returns the cost of moving from one node  to the next

    AStar* [G, N, D] = object ## \
        ## The configured A* interface \
        ## `G` is the type for the overall graph
        ## `N` is the type of element that represents nodes in the graph \
        ## `D` is the numeric type to use for priorities and costs

        graph: G ## \
            ## The graph being traversed

        heuristic: proc (a, b: N): D ## \
            ## Estimates the distance between two nodes in the graph


proc newAStar*[G: Graph, N: Node, D: Distance](
    graph: G,
    heuristic: proc(a, b: N): D
): AStar[G, N, D] =
    ## Creates a new AStar instance
    result = AStar[G, N, D]( graph: graph, heuristic: heuristic )


type
    FrontierElem[N, D] = tuple[node: N, priority: D] ## \
        ## Internally used to associate a graph node with how much it costs

    CameFrom[N, D] = tuple[node: N, cost: D] ## \
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
    ## Iterates over the nodes that connect the start and goal

    # The frontier is the list of nodes we need to visit, sorted by a
    # combination of cost and how far we estimate them to be from the goal
    var frontier =
        newHeap[FrontierElem[N, D]] do (a, b: FrontierElem[N, D]) -> int:
            return cmp(a.priority, b.priority)

    # Put the start node into the frontier so we have a place to kick off
    frontier.push( (node: start, priority: 0.0) )

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

        let currentCost: D = `[]`(cameFrom, current.node).cost

        for next in astar.graph.neighbors(current.node):

            # Adding current cost lets us track the total it took to get here
            let cost = currentCost + astar.graph.cost(current.node, next)

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

type
    Point* = concept p ## \
        ## An X/Y Coordinate
        p.x is Distance
        p.y is Distance

proc asTheCrowFlies*( a, b: Point ): float {.procvar.} =
    return sqrt(
        pow(float(a.x) - float(b.x), 2) +
        pow(float(a.y) - float(b.y), 2) )

