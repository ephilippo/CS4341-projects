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

    def __init__(self, name, avatar, x, y):
        CharacterEntity.__init__(self, name, avatar, x, y)
        self.monster_dists = {}
        self.explosion_dists = {}
        self.walldists = {}
        self.exit_dist = None


    """
    no walls       -moving
    walls          -planting bomb
    bomb/fire      -avoiding

    if no monster, just follow A*
    if monster(s), have to dodge as well as follow A*

    staticEval
    terminalTest
    get_successors
    
    check immediate area around ourselves value moves that leave more exits   
    
    

    """

    def do(self, wrld):
        x = wrld.me(self).x
        y = wrld.me(self).y
        path = self.aStar(wrld.exitcell[0], wrld.exitcell[1], wrld)
        path_set = set(path)
        #print(path)
        #self.printAStar(path_set, wrld)

        if self.monstersInPlay(wrld) > 0 and self.needExpectiMax(path, wrld): # if there are more than 0 monsters
            if len(self.wallsInWay(wrld, set(path))) > 0: # if our a* path is blocked by a wall at any point
                # need to blow up walls
                pass
            else: # otherwise move to the exit
                _, move = self.expectimax(wrld, (0, 0), 2, True)
                #print("Moves: ", move[0], move[1])
                self.move(move[0], move[1])
        else:
            # Astar to exit (no monsters)
            if len(self.wallsInWay(wrld, set(path))) > 0: # if our a* path is blocked by a wall at any point
                pass# need to blow up walls
            else: # otherwise move to the exit
                self.move(path[1][0]-x, path[1][1]-y)


    def wallsInWay(self, wrld, path_set):
        wallsInWay = set()
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.grid[i][j] and ((i, j) in path_set):
                    wallsInWay.add((i, j))
        return wallsInWay


    def needExpectiMax(self, exit_path, wrld):
        for idx, mon in wrld.monsters.items():
            mon_x = idx % wrld.width()
            mon_y = (idx-mon_x)/wrld.width()
            mon_path = self.aStar(mon_x, mon_y, wrld)
            if (len(mon_path) <= 4) and (len(mon_path) < len(exit_path)):
                return True
        return False



    def monstersInPlay(self, wrld):
        return len(wrld.monsters)


    def explosionInPlay(self, wrld):
        # TODO: check for bomb or fire(explosion) on the board
        if len(wrld.explosions):
            return True
        else:
            return False


    def bombInPlay(self, wrld):
        # TODO: check for bomb or fire(explosion) on the board
        if len(wrld.bombs):
            return True
        else:
            return False

    def staticEval(self, wrld):
        found = False
        m_dist_penalty = 0
        not_free_spaces = []
        neighbors = []
        num_monsters = 1
        for e in wrld.events:
            if e.tpe == e.BOMB_HIT_CHARACTER:
                exit = 0
                found = True
            if e.tpe == e.CHARACTER_KILLED_BY_MONSTER:
                exit = 0
                m_dist_penalty = -100000000
                found = True
            if e.tpe == e.CHARACTER_FOUND_EXIT:
                exit = 0
                found = True
        if not found:
            x = wrld.me(self).x
            y = wrld.me(self).y
            neighbors = self.neighbors((x, y), wrld)
            for i in (self.wallsInWay(wrld, set(neighbors))):
                not_free_spaces.append((i[0], i[1]))
            for i in neighbors:
                if wrld.monsters_at(i[0], i[1]):
                    not_free_spaces.append((i[0], i[1]))
            #print(free_spaces)
            m_dist_penalty = 0
            num_monsters = 0
            for monster in wrld.monsters.values():
                path = self.aStar(monster[0].x, monster[0].y, wrld)
                #path_set = set(path)
                #self.printAStar(path_set, wrld)
                dist = len(path)
                num_monsters +=1
                if dist >= 3:
                    m_dist_penalty += dist
            path = self.aStar(wrld.exitcell[0], wrld.exitcell[1], wrld)
            #path_set = set(path)
            #self.printAStar(path_set, wrld)
            exit = len(path)
        score =  .2*(30 - exit) + .65*(m_dist_penalty/num_monsters) + .15*(1/(1+len(not_free_spaces))) #.4*(30 - exit) + .5*(m_dist_penalty/num_monsters) + .1*(1/(1+len(not_free_spaces)))
        #score = 1/(1+m_dist_penalty) + 1/(1+exit) - 1/(1+len(free_spaces))
        #print("Score: ", score)
        return score


    def terminalTest(self, wrld):
        # if we are in the same place as a monster then we are dead
        for e in wrld.events:
            if e.tpe == e.BOMB_HIT_CHARACTER:
                return True
            if e.tpe == e.CHARACTER_KILLED_BY_MONSTER:
                return True
            if e.tpe == e.CHARACTER_FOUND_EXIT:
                return True
        return False

    def get_successors(self, wrld):
        """
        return all the possible next world states from the current world state
        """
        x = wrld.me(self).x
        y = wrld.me(self).y
        succ = []
        char_moves = self.neighbors((x, y), wrld)
        for val in char_moves:
            wrld.me(self).move(val[0]-x, val[1]-y)
            if wrld.monsters:
                monsters = next(iter(wrld.monsters.values()))
                for m in monsters:
                    for dx in [-1, 0, 1]:
                        # Avoid out-of-bound indexing
                        if (m.x + dx >= 0) and (m.x + dx < wrld.width()):
                            # Loop through delta y
                            for dy in [-1, 0, 1]:
                                if (dx != 0) or (dy != 0):
                                # Avoid out-of-bound indexing
                                    if (m.y + dy >= 0) and (m.y + dy < wrld.height()):
                                        #if (not (dx == 0 and dy == 0)):
                                        if not wrld.wall_at(m.x+dx, m.y+dy):
                                            # Set move in wrld
                                            m.move(dx, dy)
                                            # Get new world
                                            succ.append((wrld.next()[0], val[0]-x, val[1]-y))
            else:
                succ.append((wrld.next()[0], val[0]-x, val[1]-y))
        return succ


    def expectimax(self, wrld, col, depth, maximizingPlayer):
        #print("Depth: ", depth)
        #print("World: ")
        #wrld.printit()
        if depth == 0 or self.terminalTest(wrld):
            return self.staticEval(wrld), col
        if maximizingPlayer:
            maxEval = -10**15
            best_action = (0, 0)
            for child in self.get_successors(wrld):  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth-1, not(maximizingPlayer))
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = (child[1], child[2])  # the best move will have the best value
                '''alpha = max(alpha, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    break'''
            #print(best_action)
            #print("MaxEval: ", maxEval)
            return maxEval, best_action
        else:
            best_action = (0, 0)
            succ = self.get_successors(wrld)
            length = len(succ)
            sum_vals = 0
            for child in succ:  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth-1, not(maximizingPlayer))
                sum_vals += evalu
                '''beta = min(beta, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    break'''
            expectival = sum_vals/length
            #print("Expectival: ", expectival)
            return expectival, best_action

    def aStar(self, goal_x, goal_y, wrld):
        # A* Implementation
        path = []
        if not self.terminalTest(wrld):
            x = wrld.me(self).x
            y = wrld.me(self).y
            frontier = PriorityQueue()
            start = (x, y)
            goal = (goal_x, goal_y)
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
                #print(tracker)
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




