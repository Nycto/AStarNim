##
## Classic A-Star path finding
##
## For more information about A-Star itself, the folks over at Red Blob Games
## put together a very comprehensive introduction:
##
## http://www.redblobgames.com/pathfinding/a-star/introduction.html
##

import binaryheap, tables, hashes, math, options

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
        `hash`(n) is Hash

    Graph* = concept g
        ## The graph being traversed.
        ## * `nieghbors`: Iterates over the neighbors of a node in the graph
        ## * `cost`: Returns the price for moving between two nodes
        var node: Node
        for neighbor in g.neighbors(node):
            type(neighbor) is Node
        cost(g, node, node) is Distance

    Point* = concept p
        ## An X/Y Coordinate. This isn't used by the A-Star algorithm itself,
        ## but by the built in heuristic procs.
        p.x is Distance
        p.y is Distance


proc asTheCrowFlies*[P: Point]( node, goal: P ): float {.inline.} =
    ## A convenience function that measures the exact distance between two
    ## points. This is meant to be used as the heuristic when creating a new
    ## `AStar` instance.
    return sqrt(
        pow(float(node.x) - float(goal.x), 2) +
        pow(float(node.y) - float(goal.y), 2) )

proc manhattan*[P: Point, D: Distance](node, goal: P): D {.inline.} =
    ## A convenience function that measures the manhattan distance between two
    ## points. This is meant to be used as the heuristic when creating a new
    ## `AStar` instance.
    return D( abs(node.x - goal.x) + abs(node.y - goal.y) )

proc chebyshev*[P: Point, D: Distance](node, goal: P): D {.inline.} =
    ## A convenience function that measures the chebyshev distance between two
    ## points. This is also known as the diagonal distance. This is meant to be
    ## used as the heuristic when creating a new `AStar` instance.
    return D( max(abs(node.x - goal.x), abs(node.y - goal.y)) )

proc onLineToGoal*[P: Point, D: Distance](node, start, goal: P): D {.inline.} =
    ## Computes the cross-product between the start-to-goal vector and the
    ## current-point-to-goal vector. When these vectors don't line up, the
    ## result will be larger. This allows you to give preference to points that
    ## are along the direct line between the start and the goal.
    ##
    ## For example, you could define a heuristic like this:
    ## ```nimrod
    ## proc heuristic(grid: Grid, node, start, goal, parent: Point): float =
    ##     return 1.5 * onLineToGoal[Point, float](node, start, goal) +
    ##         asTheCrowFlies(node, goal)
    ## ```
    let dx1 = node.x - goal.x
    let dy1 = node.y - goal.y
    let dx2 = start.x - goal.x
    let dy2 = start.y - goal.y
    return D( abs(dx1 * dy2 - dx2 * dy1) )

proc straightLine*[P: Point, D: Distance](
    weight: D, node: P, grandparent: Option[P]
): D =
    ## Returns the given weight if a node doesn't have any turns. Otherwise,
    ## returns `1`. You can multiply the result of a different heuristic to
    ## give preference to straight paths.
    ##
    ## For example, you could define a heuristic like this:
    ## ```nimrod
    ## proc heuristic(
    ##     grid: StraightLineGrid,
    ##     node, start, goal, parent: XY,
    ##     grandparent: Option[XY]
    ## ): float =
    ##     straightLine[XY, float](1.2, node, grandparent) *
    ##         manhattan[XY, float](node, goal)
    ##```
    if grandparent.isSome:
        let gpNode = grandparent.get
        if gpNode.x != node.x and gpNode.y != node.y:
            return weight
    return D(1)


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

proc calcHeuristic[G: Graph, N: Node, D: Distance] (
    graph: G,
    next, start, goal: N,
    current: FrontierElem[N, D],
    cameFrom: Table[N, CameFrom[N, D]],
): D {.inline.} =
    ## Delegates the heuristic call off to the matching function
    when compiles(graph.heuristic(next, start, goal, current.node)):
        return D(graph.heuristic(next, start, goal, current.node))

    elif compiles(graph.heuristic(next, start, goal, current.node, none(N))):
        var grandparent: Option[N]
        if cameFrom.hasKey(current.node):
            grandparent = some[N]( `[]`(cameFrom, current.node).node )
        else:
            grandparent = none(N)
        return D(graph.heuristic(next, start, goal, current.node, grandparent))

    else:
        return D(graph.heuristic(next, goal))

iterator path*[G: Graph, N: Node, D: Distance](graph: G, start, goal: N): N =
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
        let current = frontier.pop

        # Now that we have a map of back-references, yield the path back out to
        # the caller
        if current.node == goal:
            for node in backtrack(cameFrom, start, goal):
                yield node
            break

        for next in graph.neighbors(current.node):

            # The cost of moving into this node from the goal
            let cost = current.cost + D( graph.cost(current.node, next) )

            # If we haven't seen this point already, or we found a cheaper
            # way to get to that
            if not cameFrom.hasKey(next) or cost < `[]`(cameFrom, next).cost:

                # Add this node to the backtrack map
                `[]=`(cameFrom, next, (node: current.node, cost: cost))

                # Estimate the priority of checking this node
                let priority: D = cost + calcHeuristic[G, N, D](
                    graph, next, start, goal, current, cameFrom )

                # Also add it to the frontier so we check out its neighbors
                frontier.push( (next, priority, cost) )


