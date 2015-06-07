##
## A-Star path finding
## @see http://www.redblobgames.com/pathfinding/a-star/introduction.html
##

import binaryheap, tables

type
    Distance* = int ## \
        ## Distance is used to measure cost and the heuristic between two points

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


proc newAStar*[G, N](
    graph: G,
    heuristic: proc(a, b: N): Distance {.closure.}
): AStar[G, N] =
    ## Creates a new AStar instance
    result = AStar[G, N]( graph: graph, heuristic: heuristic )


type
    FrontierElem[N] = tuple[node: N, cost: Distance] ## \
        ## Internally used to associate a graph node with how much it costs

    CameFrom[N] = distinct tuple[node: N, cost: Distance] ## \
        ## Given a node, this stores the node you need to backtrack to to get
        ## to this node and how much it costs to get here.  Yes, I am aware
        ## this has the same structure as FrontierElem, but they are
        ## conceptually different; hence different definitions.

iterator path*[G: Graph, N]( graph: AStar[G, N], start, goal: N ): N =
    ## Iterates over the nodes that connect the start and goal

    if start != goal:

        # The frontier is the list of nodes we need to visit, sorted by a
        # combination of cost and how far we estimate them to be from the goal
        var frontier = newHeap[FrontierElem[N]](
            proc (a, b: FrontierElem[N]): Distance =
                let aCost: Distance = a.cost
                let bCost: Distance = b.cost
                return aCost - bCost
        )

        # Put the start node into the frontier so we have a place to kick off
        frontier.push( (node: start, cost: 0) )

        # A map of backreferences. After getting to the goal, you use this to walk
        # backwards through the path and ultimately find the reverse path
        var cameFrom = newTable[N, CameFrom[N]]()

        while frontier.size > 0:
            let current = frontier.pop

