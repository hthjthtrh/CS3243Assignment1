import os
import sys
import heapq

class Puzzle(object):
    def __init__(self, init_state, goal_state):
        # You may add more attributes as necessary
        self.init_state = init_state
        self.initStateStr = self.stateToStrConvert(init_state)
        self.goal_state = goal_state
        self.goalStateStr = self.stateToStrConvert(goal_state)
        self.actions = list()
        self.solvable = False
        self.ManhattanLookUp = {
            1:(0,0),
            2:(0,1),
            3:(0,2),
            4:(1,0),
            5:(1,1),
            6:(1,2),
            7:(2,0),
            8:(2,1)
        }

    def solve(self):
        # TODO: Write your code here
        # return: a list of actions like: ["UP", "DOWN"]
        exploredStates = set()
        frontier = []

        # push initial state into frontier
        heapq.heappush(frontier,self.Node(0+self.h(self.initStateStr),self.initStateStr,[],0))

        while len(frontier) != 0:
            nodeToExplore = heapq.heappop(frontier)
            currentState = nodeToExplore.state
            currentPathCost = nodeToExplore.pathCost
            currentPath = nodeToExplore.path
            
            exploredStates.add(currentState)

            # goal test
            if self.goalTest(currentState):
                self.solvable = True
                self.actions = currentPath
                break

            validActionList = self.validActions(currentState)

            for action in validActionList:
                newState = self.stateTransition(currentState, action)
                if newState not in exploredStates:
                    newF = self.f(currentPathCost + 1, self.h(newState))
                    newPath = currentPath + [action]
                    newPathCost = currentPathCost + 1
                    newNode = self.Node(newF,newState,newPath,newPathCost)
                    heapq.heappush(frontier,newNode)

        if self.solvable:
            return self.actions
        else:
            return ['UNSOLVABLE']

    # return a list of possible actions from given state
    def validActions(self, state):
        validActions = []
        emptySpaceCoord = self.positionToCoord(state.find('0'))
        
        if emptySpaceCoord[0] != 0:
            # not top row can move down
            validActions.append('DOWN')
        if emptySpaceCoord[0] != 2:
            # not bottom row can move up
            validActions.append('UP')
        if emptySpaceCoord[1] != 0:
            # not left column can move right
            validActions.append('RIGHT')
        if emptySpaceCoord[1] != 2:
            # not right column can move left
            validActions.append('LEFT')
        return validActions

    # return the new state after action is taken on given state
    def stateTransition(self, state, action):
        # locate the 0
        newState = ''
        posToShift = -1
        emptySpacePos = state.find('0')
        if action == 'UP':
            posToShift = emptySpacePos + 3
        elif action == 'DOWN':
            posToShift = emptySpacePos - 3
        elif action == 'LEFT':
            posToShift = emptySpacePos + 1
        else:
            # right
            posToShift = emptySpacePos - 1
        if emptySpacePos < posToShift:
            # slice to emptySpacePos then replace with posToShift
            newState = state[:emptySpacePos] + state[posToShift] + state[(emptySpacePos+1):posToShift] + state[emptySpacePos] + state[(posToShift+1):]
        else:
            newState = state[:posToShift] + state[emptySpacePos] + state[(posToShift+1):emptySpacePos] + state[posToShift] + state[(emptySpacePos+1):]
        
        return newState

    # compute h(state)
    def h(self, state):
        totalManhattan = 0
        for i in range(9):
            if state[i] != '0':
                currentPos = self.positionToCoord(i)
                finalPos = self.ManhattanLookUp[int(state[i])]
                totalManhattan += abs(currentPos[0]-finalPos[0]) + abs(currentPos[1]-finalPos[1])
        return totalManhattan     

    def f(self, pathCost, h):
        return pathCost + h

    def goalTest(self, state):
        return state == self.goalStateStr

    # convert state in the form of matrix into single string
    def stateToStrConvert(self, stateList):
        stateStr = ''
        for row in stateList:
            for column in row:
                stateStr += str(column)
        return stateStr

    # convert position in an array 0 - 8 into coordinate in matrix (x,y)
    # e.g. 4 into (1,1)
    def positionToCoord(self, arrayPstion):
        return (arrayPstion//3, arrayPstion%3)

    # opposite of positionToCoord()
    def coordToPosition(self, coord):
        return coord[0] * 3 + coord[1]

    # node used in the heap, __lt__ defined such that heapq can maintain the heap correctly
    class Node(object):
        def __init__(self, f, state, path, pathCost):
            self.f = f
            self.state = state
            self.path = path
            self.pathCost = pathCost

        def __lt__(self, other):
            return self.f < other.f  



    # You may add more (helper) methods if necessary.
    # Note that our evaluation scripts only call the solve method.
    # Any other methods that you write should be used within the solve() method.

def printMatrix(state, action):
    if action is not None:
        print(action)
    withSpace = state.replace('0',' ')
    for i in range(3):
        print(withSpace[i*3]+' '+withSpace[i*3+1]+' '+withSpace[i*3+2])

if __name__ == "__main__":
    # do NOT modify below
    if len(sys.argv) != 3:
        raise ValueError("Wrong number of arguments!")

    try:
        f = open(sys.argv[1], 'r')
    except IOError:
        raise IOError("Input file not found!")

    init_state = [[0 for i in range(3)] for j in range(3)]
    goal_state = [[0 for i in range(3)] for j in range(3)]
    lines = f.readlines()

    i,j = 0, 0
    for line in lines:
        for number in line:
            if '0'<= number <= '8':
                init_state[i][j] = int(number)
                j += 1
                if j == 3:
                    i += 1
                    j = 0

    for i in range(1, 9):
        goal_state[(i-1)//3][(i-1)%3] = i
    goal_state[2][2] = 0

    puzzle = Puzzle(init_state, goal_state)
    ans = puzzle.solve()

    state = puzzle.initStateStr
    printMatrix(state, None)
    for step in ans:
        newState = puzzle.stateTransition(state, step)
        printMatrix(newState, step)
        state = newState

    with open(sys.argv[2], 'a') as f:
        for answer in ans:
            f.write(answer+'\n')

