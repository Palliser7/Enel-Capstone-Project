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

from locale import currency
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
    return  ["South", s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"

    from util import Stack                                                  #importing stack class for LIFO

    startPosition = problem.getStartState()                                 #getting the start position
    currentPosition = problem.getStartState()                               #getting current position
    exploredPositions = []                                                  #list for explored positions
    exploredPositions.append(currentPosition)                               #adding start position to explored positions
    fringe = Stack()                                                        #creating lifo fringe
    tupleHolder = (startPosition, [])                                       #creating a tuple to store position and the directions to get to that position
    fringe.push(tupleHolder)                                                #pushing the start position onto the fringe

    while problem.isGoalState(currentPosition) is False:                    #loop that exits when goal is found
        temp = fringe.pop()                                                 #pops the next position off of the fringe
        currentPosition = temp[0]                                           #stores current position
        directions = temp[1]                                                #stores directions to current position
        exploredPositions.append(currentPosition)                           #adds the current position to the explored position list
        nextPositions = problem.getSuccessors(currentPosition)              #gets the next available positions to explore

        for position in nextPositions:                                      #loops through next available positions to explore
            coordinate = position[0]                                        #gets the coordinates from the next available positions
            if coordinate not in exploredPositions:                         #loop for checking if next explorable position has already been explored
                currentPosition = position[0]                               #sets this as the next tile to explore
                tempDirection = position[1]                                 #sets the direction to the tile as a temp direction
                fringe.push((coordinate, directions + [tempDirection]))     #pushes the position and the current directions and the new direction to that position


    return directions + [tempDirection]                                     #return the current directions plus the direction to get to the goal tile


    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"

    from util import Queue                                                  #importing queue class for FIFO

    startPosition = problem.getStartState()                                 #getting the start position
    currentPosition = problem.getStartState()                               #getting current position
    exploredPositions = []                                                  #list for explored positions
    exploredPositions.append(currentPosition)                               #adding start position to explored positions
    fringe = Queue()                                                        #creating fifo fringe
    tupleHolder = (startPosition, [])                                       #creating a tuple to store position and the directions to get to that position
    fringe.push(tupleHolder)                                                #pushing the start position onto the fringe

    while problem.isGoalState(currentPosition) is False:                    #loop that exits when goal is found
        temp = fringe.pop()                                                 #pops the next position off of the fringe
        currentPosition = temp[0]                                           #stores current position
        directions = temp[1]                                                #stores directions to current position
        exploredPositions.append(currentPosition)                           #adds the current position to the explored position list
        nextPositions = problem.getSuccessors(currentPosition)              #gets the next available positions to explore

        for position in nextPositions:                                      #loops through next available positions to explore
            coordinate = position[0]                                        #gets the coordinates from the next available positions
            if coordinate not in exploredPositions:                         #loop for checking if next explorable position has already been explored
                currentPosition = position[0]                               #sets this as the next tile to explore
                tempDirection = position[1]                                 #sets the direction to the tile as a temp direction
                fringe.push((coordinate, directions + [tempDirection]))     #pushes the position and the current directions and the new direction to that position

    return directions + [tempDirection]                                     #return the current directions plus the direction to get to the goal tile

    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    
    from util import PriorityQueue                                          #importing priority queue class for uniformcost search

    startPosition = problem.getStartState()                                 #getting the start position
    currentPosition = problem.getStartState()                               #getting current position
    exploredPositions = []                                                  #list for explored positions
    exploredPositions.append(currentPosition)                               #adding start position to explored positions
    fringe = PriorityQueue()                                                #creating uniformcost fringe
    tupleHolder = (startPosition, [])                                       #creating a tuple to store position and the directions to get to that position
    fringe.push(tupleHolder, 0)                                             #pushing the start position onto the fringe with a cost of 0

    while problem.isGoalState(currentPosition) is False:                    #loop that exits when goal is found
        temp = fringe.pop()                                                 #pops the next position off of the fringe
        currentPosition = temp[0]                                           #stores current position
        directions = temp[1]                                                #stores directions to current position
        exploredPositions.append(currentPosition)                           #adds the current position to the explored position list
        nextPositions = problem.getSuccessors(currentPosition)              #gets the next available positions to explore

        for position in nextPositions:                                      #loops through next available positions to explore
            coordinate = position[0]                                        #gets the coordinates from the next available positions
            if coordinate not in exploredPositions:                         #loop for checking if next explorable position has already been explored
                currentPosition = position[0]                               #sets this as the next tile to explore
                tempDirection = position[1]                                 #sets the direction to the tile as a temp direction
                fringe.push((coordinate, directions + [tempDirection]), problem.getCostOfActions(directions + [tempDirection]))     #pushes the position and the current directions and the new direction to that position as well as the new cost to that tile

    return directions + [tempDirection]                                     #return the current directions plus the direction to get to the goal tile

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

    from util import PriorityQueue                                          #importing priority queue class for uniformcost search

    startPosition = problem.getStartState()                                 #getting the start position
    currentPosition = problem.getStartState()                               #getting current position
    exploredPositions = []                                                  #list for explored positions
    exploredPositions.append(currentPosition)                               #adding start position to explored positions
    fringe = PriorityQueue()                                                #creating uniformcost fringe
    tupleHolder = (startPosition, [])                                       #creating a tuple to store position and the directions to get to that position
    fringe.push(tupleHolder, nullHeuristic(startPosition, problem))        #pushing the start position onto the fringe with a cost of calculated by the nullheristic

    while problem.isGoalState(currentPosition) is False:                    #loop that exits when goal is found
        temp = fringe.pop()                                                 #pops the next position off of the fringe
        currentPosition = temp[0]                                           #stores current position
        directions = temp[1]                                                #stores directions to current position
        exploredPositions.append(currentPosition)                           #adds the current position to the explored position list
        nextPositions = problem.getSuccessors(currentPosition)              #gets the next available positions to explore

        for position in nextPositions:                                      #loops through next available positions to explore
            coordinate = position[0]                                        #gets the coordinates from the next available positions
            if coordinate not in exploredPositions:                         #loop for checking if next explorable position has already been explored
                currentPosition = position[0]                               #sets this as the next tile to explore
                tempDirection = position[1]                                 #sets the direction to the tile as a temp direction
                fringe.push((coordinate, directions + [tempDirection]), (problem.getCostOfActions(directions + [tempDirection]) + heuristic(coordinate, problem)))     #pushes the position and the current directions and the new direction to that position as well as the new cost to that tile added with the heuristics
    
    return directions + [tempDirection]

    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
