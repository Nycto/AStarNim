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
        ## * `cost`: Returns the price for moving between two nodes
        neighbors(g, Node) is iterator Node
        cost(g, Node, Node) is Distance

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

proc onLineToGoal*[P: Point, D: Distance](
    weight: D, inner: proc (a, b: P): D
): proc (node, start, goal, cameFrom: P): D =
    ## Modifies another heuristic to add weight to points that are closer to
    ## the line from the start to the goal. The proc returned from this proc is
    ## meant to be used as the heuristic when creating a new `AStar` instance
    return proc (node, start, goal, cameFrom: P): D =
        let dx1 = node.x - goal.x
        let dy1 = node.y - goal.y
        let dx2 = start.x - goal.x
        let dy2 = start.y - goal.y
        let crossProduct = D( abs(dx1 * dy2 - dx2 * dy1) )
        return inner(node, goal) + (weight * crossProduct)


type
    FrontierElem[N, D] = tuple[node: N, priority: D, cost: D]
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

template astarAlgo[G: Graph, N: Node, D: Distance](
    heuristic: expr
): N {.immediate.} =
    ## Executes the A-Star algorithm and iterates over the nodes that connect
    ## the start and goal

    # The frontier is the list of nodes we need to visit, sorted by a
    # combination of cost and how far we estimate them to be from the goal
    var frontier =
        newHeap[FrontierElem[N, D]] do (a, b: FrontierElem[N, D]) -> int:
            return cmp(a.priority, b.priority)

    # Put the start node into the frontier so we have a place to kick off
    frontier.push( (node: start, priority: D(0), cost: D(0)) )

    # A map of backreferences. After getting to the goal, you use this to walk
    # backwards through the path and ultimately find the reverse path
    var cameFrom = initTable[N, CameFrom[N, D]]()

    while frontier.size > 0:
        let current {.inject.} = frontier.pop

        # Now that we have a map of back-references, yield the path back out to
        # the caller
        if current.node == goal:
            for node in backtrack(cameFrom, start, goal):
                yield node
            break

        for next {.inject.} in graph.neighbors(current.node):

            # The cost of moving into this node from the goal
            let cost = current.cost + D( graph.cost(current.node, next) )

            # If we haven't seen this point already, or we found a cheaper
            # way to get to that
            if not cameFrom.hasKey(next) or cost < `[]`(cameFrom, next).cost:

                # Add this node to the backtrack map
                `[]=`(cameFrom, next, (node: current.node, cost: cost))

                # Estimate the priority of checking this node
                let priority: D = cost + heuristic

                # Also add it to the frontier so we check out its neighbors
                frontier.push( (next, priority, cost) )

iterator astar*[G: Graph, N: Node, D: Distance](
    graph: G, start, goal: N,
    heuristic: proc (node, goal: N): D
): N =
    ## Runs A* between two points against the given graph. Heuristic is passed
    ## the current node and the goal
    astarAlgo( heuristic(next, goal) )

iterator astar*[G: Graph, N: Node, D: Distance](
    graph: G, start, goal: N,
    heuristic: proc (node, start, goal, cameFrom: N): D
): N =
    ## Runs A* between two points against the given graph. Heuristic is passed
    ## the current node, the start, the goal, and the previous node
    astarAlgo( heuristic(next, start, goal, current.node) )

