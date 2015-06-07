##
## A-Star path finding
## @see http://www.redblobgames.com/pathfinding/a-star/introduction.html
##

import binaryheap, tables, hashes

type
    Distance* = int ## \
        ## Distance is used to measure cost and the heuristic between two points

    Node* = concept n ## \
        ## Represents a node stored within a graph

        `==`(n, n) is bool ## \
            ## Nodes must be comparable so we know when we reach the goal

        `hash`(n) is THash ## \
            ## Nodes are used as keys in a table, so they need to be hashable

    Graph*[N] = concept g ## \
        ## The graph being traversed

        neighbors(g, N) is iterator N ## \
            ## Allows iteration over the neighbors of a node in the graph

        cost(g, N, N) is Distance ## \
            ## Returns the cost of moving from one node  to the next

    AStar* [G, N] = object ## \
        ## The configured A* interface \
        ## `G` is the type for the overall graph
        ## `N` is the type of element that represents nodes in the graph \

        graph: G ## \
            ## The graph being traversed

        heuristic: proc (a, b: N): Distance ## \
            ## Estimates the distance between two nodes in the graph


proc newAStar*[G: Graph, N: Node](
    graph: G,
    heuristic: proc(a, b: N): Distance {.closure.}
): AStar[G, N] =
    ## Creates a new AStar instance
    result = AStar[G, N]( graph: graph, heuristic: heuristic )


type
    FrontierElem[N] = tuple[node: N, priority: Distance] ## \
        ## Internally used to associate a graph node with how much it costs

    CameFrom[N] = tuple[node: N, cost: Distance] ## \
        ## Given a node, this stores the node you need to backtrack to to get
        ## to this node and how much it costs to get here


iterator backtrack[N]( cameFrom: Table[N, CameFrom[N]], start, goal: N ): N =
    ## Once the table of back-references is filled, this yields the reversed
    ## path back to the consumer
    yield start

    var current: N = goal
    var path: seq[N] = @[]

    while current != start:
        path.add(current)
        current = `[]`(cameFrom, current).node

    if path.len > 0:
        for i in (path.len - 1)..0:
            yield path[i]

iterator path*[G: Graph, N: Node]( astar: AStar[G, N], start, goal: N ): N =
    ## Iterates over the nodes that connect the start and goal

    # The frontier is the list of nodes we need to visit, sorted by a
    # combination of cost and how far we estimate them to be from the goal
    var frontier =
        newHeap[FrontierElem[N]] do (a, b: FrontierElem[N]) -> Distance:
            return a.priority - b.priority

    # Put the start node into the frontier so we have a place to kick off
    frontier.push( (node: start, priority: 0) )

    # A map of backreferences. After getting to the goal, you use this to walk
    # backwards through the path and ultimately find the reverse path
    var cameFrom = initTable[N, CameFrom[N]]()

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

