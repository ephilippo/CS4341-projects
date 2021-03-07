# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back, Style

# my imports
from queue import PriorityQueue
import math


class TestCharacter(CharacterEntity):

    """
    no walls       -moving
    walls          -planting bomb
    bomb/fire      -avoiding

    if no monster, just follow A*
    if monster(s), have to dodge as well as follow A*
    """

    def expectimax(self, brd, col, depth, alpha, beta, maximizingPlayer):
        if depth == 0 or self.terminalTest(brd):
            return self.staticEval(brd, col), col
        if maximizingPlayer:
            maxEval = -10**15
            best_action = col
            for child in self.get_successors(brd):  # getting the children of the current node
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = child[1]  # the best move will have the best value
                alpha = max(alpha, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    break
            return maxEval, best_action
        else:
            expectival = 0
            best_action = col
            for child in self.get_successors(brd):  # getting the children of the current node
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                if min(minEval, evalu) < minEval:
                    minEval = evalu
                    best_action = child[1]  # the best move will have the best value
                beta = min(beta, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    break
            return minEval, best_action

    #def states:


    def do(self, wrld):
        '''if state1:
            #stuff'''


        path = self.aStar(wrld.exitcell[0], wrld.exitcell[1], wrld)
        path_set = set(path)
        print(path)
        self.move(path[1][0]-self.x, path[1][1]-self.y)
        self.printAStar(path_set, wrld)
        wallsInWay = set()
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.grid[i][j] and ((i, j) in path_set):
                    wallsInWay.add((i, j))

        print(wrld.grid)


    def bombInPlay(self, wrld):
        # TODO: check for bomb or fire(explosion) on the board
        pass


    def aStar(self, x, y, wrld):
        # A* Implementation
        frontier = PriorityQueue()
        start = (self.x, self.y)
        goal = (x, y)
        frontier.put((0, start))
        came_from = dict()
        cost_so_far = dict()
        came_from[start] = None
        cost_so_far[start] = 0
        while not frontier.empty():
            current = frontier.get()[1]
            if current == goal:
                break
            for next in self.neighbors(current, wrld):
                new_cost = cost_so_far[current] + self.cost(next, wrld)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(goal, next)
                    frontier.put((priority, next))
                    came_from[next] = current
        path = [goal]
        tracker = goal
        while came_from[tracker]:
            tracker = came_from[tracker]
            path.append(tracker)
            print(tracker)
        path.reverse()
        return path

    def heuristic(self, goal, next):
        return math.sqrt((goal[0] - next[0]) ** 2 + (goal[1] - next[1]) ** 2)

    def cost(self, next, wrld):
        if wrld.wall_at(next[0], next[1]):
            return wrld.bomb_time + wrld.expl_duration
        else:
            return 1

    def neighbors(self, current, wrld):
        neighbors = []
        for dx in [-1, 0, 1]:
            # Avoid out-of-bound indexing
            if (current[0] + dx >= 0) and (current[0] + dx < wrld.width()):
                # Loop through delta y
                for dy in [-1, 0, 1]:
                    # Avoid out-of-bound indexing
                    if (current[1] + dy >= 0) and (current[1] + dy < wrld.height()):
                        if (not (dx == 0 and dy == 0)):
                            neighbors.append((current[0] + dx, current[1] + dy))
        return neighbors

    def printAStar(self, path_set, wrld):
        border = "+" + "-" * wrld.width() + "+\n"
        sys.stdout.write(border)
        for y in range(wrld.height()):
            sys.stdout.write("|")
            for x in range(wrld.width()):
                if wrld.characters_at(x,y):
                    for c in wrld.characters_at(x,y):
                        sys.stdout.write(Back.GREEN + c.avatar)
                elif wrld.monsters_at(x,y):
                    for m in wrld.monsters_at(x,y):
                        sys.stdout.write(Back.BLUE + m.avatar)
                elif wrld.exit_at(x,y):
                    sys.stdout.write(Back.YELLOW + "#")
                elif wrld.bomb_at(x,y):
                    sys.stdout.write(Back.MAGENTA + "@")
                elif wrld.explosion_at(x,y):
                    sys.stdout.write(Fore.RED + "*")
                elif wrld.wall_at(x,y):
                    sys.stdout.write(Back.WHITE + " ")
                elif (x, y) in path_set:
                    sys.stdout.write(Back.BLUE + " ")
                else:
                    tile = False
                    for k,clist in wrld.characters.items():
                        for c in clist:
                            if c.tiles.get((x,y)):
                                sys.stdout.write(c.tiles[(x,y)] + ".")
                                tile = True
                                break
                    if not tile:
                        sys.stdout.write(" ")
                sys.stdout.write(Style.RESET_ALL)
            sys.stdout.write("|\n")
        sys.stdout.write(border)
        sys.stdout.flush()