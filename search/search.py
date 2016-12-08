# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    # Here we implement the depthFirstSearch by first creating a stack, 2 lists and a dictionary.
    # The stack is the fringe that contains the still unexpanded nodes (that can be visited).
    # The list closed contains the nodes that have been visited.
    # The list actions contains the results.
    # The dictionary dict contains all the child nodes with the parent node as a reference.
    # The while is used to walk through the fringe to check for every node if it's not in the list closed.
    # If it's not, it will check then for every node if it's the goalstate with the function isGoalstate().
    # If it is, the node will be added to the list called actions if it's not equal to the startState.
    # The node will also be added to the dictionary.
    # If it's not the goalstate, the node will be added to the list closed and there will be checked if it has
    # successors.
    # If it doesn't have any successors,the next node in the fringe will be popped from the fringe and do the process.
    # If it does, the successors will be pushed on the stack fringe and added to the dictionary dict.
    startState = (problem.getStartState(), 0, 0)
    fringe = util.Stack()
    closed = []
    dict = {}
    actions = []
    fringe.push(startState)
          
    while not fringe.isEmpty():
        node = fringe.pop()

        if node[0] not in closed:

            if problem.isGoalState(node[0]):
                current = node
                while (current != startState):
                    actions.append(current[1])
                    current = dict[current] 
                actions.reverse()     
                return actions
        
            else:
                closed.append(node[0])
                succ = problem.getSuccessors(node[0])
                if not succ:
                    continue
                for s in succ:
                    fringe.push(s)
                    dict[s] = node
                    
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # Here we implement the breadthFirstSearch by first creating a queue, 2 lists and a dictionary.
    # The stack is the fringe that contains the still unexpanded nodes (that can be visited).
    # The list closed contains the nodes that have been visited.
    # The list actions contains the results.
    # The dictionary dict contains all the child nodes with the parent node as a reference.
    # The while is used to walk through the fringe to check for every node if it's not in the list closed.
    # If it's not, it will check then for every node if it's the goalstate with the function isGoalstate().
    # If it is, the node will be added to the list called actions if it's not equal to the startState.
    # The node will also be added to the dictionary.
    # If it's not the goalstate, the node will be added to the list closed and there will be checked if it has successors.
    # If it doesn't have any successors, the next node in the fringe will be popped from the fringe and do the process.
    # If it does, the successors will be pushed on the queue fringe and added to the dictionary dict if the key and the
    #  value of the node aren't in the dictionary.
    startState = (problem.getStartState(), 0, 0)
    fringe = util.Queue()
    closed = []
    dict = {}
    actions = []
    fringe.push(startState)   
        
    while not fringe.isEmpty():
        node = fringe.pop()
        
        if node[0] not in closed:
            
            if problem.isGoalState(node[0]):
                
                current = node
                while (current != startState):
                    actions.append(current[1])
                    current = dict[current] 
                actions.reverse()     
                return actions
        
            else:
                closed.append(node[0])
                succ = problem.getSuccessors(node[0])
                if not succ:
                    continue
                for s in succ:
                    if node in dict.keys() and s in dict.values():
                        continue
                    fringe.push(s)
                    dict[s] = node
    
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
      # Here we implement the uniformCostSearch by first creating a priority queue, 2 lists and 2 dictionaries.
    # The priority queue is the fringe that contains the still unexpanded nodes (that can be visited) ordered by the
    # least costpath.
    # The list closed contains the nodes that have been visited.
    # The list actions contains the results.
    # The dictionary path_cost contains the cost of the path.
    # The dictionary dict contains all the child nodes with the parent node as a reference.
    # It checks the same conditions as DFS and BFS. The only difference is that in the else-statement it calculates the
    # the current (up to now) cost path and push the node and cost path on the fringe if the node has successors.
    startState = (problem.getStartState(), 0, 0)
    fringe = util.PriorityQueue()
    closed = []
    dict = {}
    path_cost = {}
    path_cost[startState] = startState[2]
    actions = []
    fringe.push(startState, 0)

    while not fringe.isEmpty():
        node = fringe.pop()

        if node[0] not in closed:

            if problem.isGoalState(node[0]):
                current = node
                while (current != startState):
                    actions.append(current[1])
                    current = dict[current]
                actions.reverse()
                return actions

            else:
                closed.append(node[0])
                succ = problem.getSuccessors(node[0])
                if not succ:
                    continue
                for s in succ:
                    cost = path_cost[node] + s[2]
                    fringe.push(s, cost)
                    if node in dict.keys() and s in dict.values():
                        continue
                    dict[s] = node
                    path_cost[s] = cost
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
     # Here we implement the aStarSearch by first creating a priority queue, 2 lists and 2 dictionaries.
    # The priority queue is the fringe that contains the still unexpanded nodes (that can be visited) ordered by the
    # least estimated total cost of path through a specific node to goal(=f).
    # The list closed contains the nodes that have been visited.
    # The list actions contains the results.
    # The dictionary path_cost contains the cost of the path.
    # The dictionary dict contains all the child nodes with the parent node as a reference.
    # It checks the same conditions as UCS. The only difference is that in the else-statement it calculates the
    # the current (up to now) cost path (= g) to calculate the cost of f (= g + h(given heuristic)) and then pushes f and the node on
    # the fringe if the node has successors.
    # It also puts the new value of cost in the path_cost list to update the list.
    startState = (problem.getStartState(), 0, 0)
    fringe = util.PriorityQueue()
    closed = []
    dict = {}
    path_cost = {}
    path_cost[startState] = startState[2]
    actions = []
    fringe.push(startState, 0)


    while not fringe.isEmpty():
        node = fringe.pop()

        if node[0] not in closed:

            if problem.isGoalState(node[0]):
                current = node
                while (current != startState):
                    actions.append(current[1])
                    current = dict[current]
                actions.reverse()
                return actions

            else:
                closed.append(node[0])
                succ = problem.getSuccessors(node[0])
                if not succ:
                    continue
                for s in succ:
                    cost = path_cost[node] + s[2]
                    fcost = cost + heuristic(s[0], problem)
                    fringe.push(s, fcost)
                    if node in dict.keys() and s in dict.values():
                        continue
                    dict[s] = node
                    path_cost[s] = cost
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
