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
                    fringe.push(s)
                    dict[s] = node
    
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    startState = problem.getStartState()
    fringe = util.PriorityQueue()
    closed = [startState]
    dict = {}
    path_cost = {}
    path_cost[startState] = 0
    actions = []

    for x in problem.getSuccessors(startState):
        fringe.push(x, x[2])
        dict[x] = startState
        path_cost[x] = path_cost[startState] + x[2]


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
                    fringe.push(s, path_cost[node] + s[2])
                    if node in dict.keys() and s in dict.values():
                        continue
                    dict[s] = node
                    path_cost[s] = path_cost[node] + s[2]
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
    
     fringe = util.PriorityQueue()
    actions = []
    totnucost = {}
    camefrom = {}
    closed =[]
    curr = (problem.getStartState(), 0, 0)

    totnucost[curr[0]] = 0
    camefrom[curr[0]] = None
    fringe.push(curr, 0)

    while not fringe.isEmpty():
       node = fringe.pop()
       if node[0] not in closed:


        if problem.isGoalState(node[0]):
            current = node

            while (current[0] != problem.getStartState()):

                actions.append(current[1])
                current = camefrom[current[0]]

            actions.reverse()
            return actions
        else:
            closed.append(node)
            succ = problem.getSuccessors(node[0])
            if not succ:
                continue

            for s in succ:
                print totnucost[node[0]]
                newcost = totnucost[node[0]] + s[2]

                if s[0] not in totnucost or newcost < totnucost[node[0]]:
                    totnucost[s[0]] = newcost
                    f = newcost + heuristic(s[0], problem)
                    fringe.push(s, f)
                    if node in camefrom.keys() and s in camefrom.values():
                        continue
                    camefrom[s[0]] = node

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
