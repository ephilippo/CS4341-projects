# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back, Style

# my imports
from queue import PriorityQueue
import math

# Current score .7*(30 - exit_dist) + 2.3*(m_dist_penalty/num_monsters) + .9*(len(neighbors)-len(not_free_spaces))

class TestCharacter(CharacterEntity):

    def __init__(self, name, avatar, x, y):
        CharacterEntity.__init__(self, name, avatar, x, y)
        self.wait = 0
        self.times = 0


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
    
    what are things that are important to our character
    
    always 2-3 moves from monster
    its okay to cede ground
    mon dist to exit > our dist to exit + 1
    staying away from the sides
    

    """

    def bombScript(self):
        if self.wait < 3:
            self.move(0, 1)
            self.wait +=1
        elif self.wait == 3:
            self.place_bomb()
            self.move(1, -1)
            self.wait += 1
        elif self.wait > 3:
            self.move(0, 0)
            self.wait += 1
        if self.wait == 17:
            self.wait = 0
            self.times +=1
        if self.times == 4:
            self.wait = 18







    def do(self, wrld):
        x = wrld.me(self).x
        y = wrld.me(self).y
        char_exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)
        '''path_set = set(char_exit_path)
        print(char_exit_path)
        self.printAStar(path_set, wrld)'''
        #self.bombScript()

        if self.monstersInPlay(wrld) == 0:
            if len(self.wallsInWay(wrld, set(char_exit_path))) == 0:
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
            else:
                pass
            '''elif self.monstersInPlay(wrld) == 0:
            mon = next(iter(wrld.monsters.values()))
            mon_x = mon[0].x
            mon_y = mon[0].y
            mon_name = mon[0].name
            mon_exit_path = self.aStar(mon_x, mon_y, wrld.exitcell[0], wrld.exitcell[1], wrld)
            char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)
            char_escape = self.aStar(x, y, 0, 0, wrld)
            if y >= (mon_y + 2):
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
            elif len(char_exit_path) < len(mon_exit_path) and (y > mon_y):
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
            elif len(char_to_mon) <= 5:
                if len(char_to_mon) <= 3:
                    alpha = -10000
                    beta = 1000
                    _, move = self.expectimax(wrld, (0, 0), 2, True, alpha, beta, True, [mon])
                    self.move(move[0], move[1])
                else:
                    alpha = -10000
                    beta = 1000
                    _, move = self.expectimax(wrld, (0, 0), 3, True, alpha, beta, False, [mon])
                    self.move(move[0], move[1])
            else:
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)'''
        else:
            monsters = next(iter(wrld.monsters.values()))
            within_range = []
            all_above = True
            minimax = False
            expectimax = False
            for mon in monsters:
                mon_x = mon.x
                mon_y = mon.y
                mon_exit_path = self.aStar(mon_x, mon_y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)

                if not (y >= (mon_y + 2)):
                    all_above = False
                if not (len(char_exit_path) < len(mon_exit_path) and (y > mon_y)):
                    all_above = False
                if len(char_to_mon) <= 5:
                    within_range.append(mon)
                    if len(char_to_mon) <= 3:
                        minimax = True
                    else:
                        expectimax = True

            if all_above:
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
            elif minimax:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 3, True, alpha, beta, True, within_range)
                self.move(move[0], move[1])
            elif expectimax:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 3, True, alpha, beta, False, within_range)
                self.move(move[0], move[1])
            else:
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)




    def wallsInWay(self, wrld, path_set):
        wallsInWay = set()
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.grid[i][j] and ((i, j) in path_set):
                    wallsInWay.add((i, j))
        return wallsInWay


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

    def staticEval(self, wrld, monsters):
        found = False
        for e in wrld.events:
            if e.tpe == e.BOMB_HIT_CHARACTER:
                score = -1000
                found = True
            if e.tpe == e.CHARACTER_KILLED_BY_MONSTER:
                score = -1000
                found = True
            if e.tpe == e.CHARACTER_FOUND_EXIT:
                score = 1000
                found = True
        if not found:
            x = wrld.me(self).x
            y = wrld.me(self).y
            col_value = [-100, -10, 1, 4, 4, 1, -10, -100]
            score = col_value[x]
            #print("Initial: ", score)
            #mon = next(iter(wrld.monsters.values()))
            for mon in monsters:
                mon_x = mon.x
                mon_y = mon.y
                mon_name = mon.name
                mon_path = self.aStar(x, y, mon_x, mon_y, wrld)
                char_exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                #mon_exit_path = self.aStar(mon_x, mon_y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                if mon_name == "stupid":
                    safe_dist = 4
                else:
                    safe_dist = 5
                if len(mon_path) >= safe_dist:
                    score += 120
                    score -= len(char_exit_path)
                else:
                    score -= 300


            '''wrld.printit()
            print("Character Stuff: x:", x, "y: ", y, "mon_x: ", mon_x, "mon_y: ", mon_y)
            print("Final: ", score)
            input("Press Enter to continue getting scores...")'''

        '''if not maximizingPlayer:
            score = -score'''
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

    def get_successors(self, wrld, maximizingPlayer, monsters):
        """
        return all the possible next world states from the current world state
        """
        succ = []
        x = wrld.me(self).x
        y = wrld.me(self).y
        if maximizingPlayer:
            #monsters = next(iter(wrld.monsters.values()))
            for m in monsters:
                m.move(0, 0)
            char_moves = self.neighbors((x, y), wrld)
            print(char_moves)
            for val in char_moves:
                if not wrld.wall_at(val[0], val[1]):
                    wrld.me(self).move(val[0]-x, val[1]-y)
                    succ.append((wrld.next()[0], val[0]-x, val[1]-y))
        else:
            wrld.me(self).move(0, 0)
            #monsters = next(iter(wrld.monsters.values()))
            for m in monsters:
                mon_path = self.aStar(x, y, m.x, m.y, wrld)
                if m.name == "stupid" or len(mon_path) > 4:
                    #print("I'm with stupid")
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
                                            succ.append((wrld.next()[0], dx, dy))
                else:
                    #print("I'm with angry")
                    mon_path.reverse()
                    dx = mon_path[1][0] - m.x
                    dy = mon_path[1][1] - m.y
                    if (m.y + dy >= 0) and (m.y + dy < wrld.height()) and (m.x + dx >= 0) and (m.x + dx < wrld.width()):
                        #if (not (dx == 0 and dy == 0)):
                        if not wrld.wall_at(m.x+dx, m.y+dy):
                            # Set move in wrld
                            m.move(dx, dy)
                            # Get new world
                            succ.append((wrld.next()[0], dx, dy))
        return succ

    def expectimax(self, wrld, col, depth, maximizingPlayer, alpha, beta, mini, monsters):
        if depth == 0 or self.terminalTest(wrld):
            return self.staticEval(wrld, monsters), col
        if maximizingPlayer:
            maxEval = -10**15
            best_action = (0, 0)
            for child in self.get_successors(wrld, maximizingPlayer, monsters):  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth-1, not(maximizingPlayer), alpha, beta, mini, monsters)
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = (child[1], child[2])  # the best move will have the best value
                alpha = max(alpha, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    print("pruned", beta, alpha, maxEval, maximizingPlayer)
                    break
                print(depth, maxEval, evalu, child[1], child[2], maximizingPlayer, maxEval, best_action[0], best_action[1])
            '''wrld.printit()
            print(maxEval)
            print('maximizing node')
            input("Press Enter to continue getting scores...")'''
            return maxEval, best_action
        else:
            best_action = (0, 0)
            succ = self.get_successors(wrld, maximizingPlayer, monsters)
            length = 0
            sum_vals = 0
            minEval = 1000
            for child in succ:  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth-1, not(maximizingPlayer), alpha, beta, mini, monsters)
                print(depth, evalu, child[1], child[2], maximizingPlayer)
                if mini:
                    if min(minEval, evalu) < minEval:
                        minEval = evalu  # the best move will have the best value
                    beta = min(beta, evalu)  # alpha-beta pruning
                    if beta <= alpha:
                        print("pruned", beta, alpha, minEval, maximizingPlayer)
                        break
                else:
                    sum_vals += evalu
                    length += 1
                    beta = 1000*(len(succ)-length) + sum_vals # alpha-beta pruning
                    if beta <= alpha:
                        print("pruned", beta, alpha, len(succ), length, sum_vals, maximizingPlayer)
                        break
            if mini:
                return_val = minEval
            else:
                return_val = sum_vals/length
            return return_val, best_action


    def aStar(self, x, y, goal_x, goal_y, wrld):
        # A* Implementation
        path = []
        if not self.terminalTest(wrld):
            '''x = wrld.me(self).x
            y = wrld.me(self).y'''
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
            cost = 10
        else:
            cost = 1
        return cost

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
                    sys.stdout.write(Back.RED + " ")
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

'''x = wrld.me(self).x
            y = wrld.me(self).y
            col_value = [-100, -10, 1, 4, 4, 1, -10, -100]
            score = col_value[x]
            #print("Initial: ", score)
            #mon = next(iter(wrld.monsters.values()))
            for mon in monsters:
                mon_x = mon.x
                mon_y = mon.y
                mon_name = mon.name
                mon_path = self.aStar(x, y, mon_x, mon_y, wrld)
                char_exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                #mon_exit_path = self.aStar(mon_x, mon_y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                if mon_name == "stupid":
                    safe_dist = 4
                else:
                    safe_dist = 5
                if len(mon_path) >= safe_dist:
                    score += 120
                    score -= len(char_exit_path)
                else:
                    score -= 300'''







''' x = wrld.me(self).x
            y = wrld.me(self).y
            neighbors = self.neighbors((x, y), wrld)

            exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)
            for i in (self.wallsInWay(wrld, set(neighbors))):
                not_free_spaces.append((i[0], i[1]))
            for i in neighbors:
                if wrld.monsters_at(i[0], i[1]):
                    not_free_spaces.append((i[0], i[1]))
            #print(free_spaces)
            m_dist_penalty = 0
            num_monsters = 0
            for idx, mon in wrld.monsters.items():
                mon_x = idx % wrld.width()
                mon_y = int((idx-mon_x)/wrld.width())
                mon_path = self.aStar(x, y, mon_x, mon_y, wrld)
                num_monsters += 1
                dist = len(mon_path)
                if (len(mon_path) >= 3):
                    m_dist_penalty += 10

            if num_monsters == 0:
                num_monsters = 1
            exit_dist = len(exit_path)

        exit_bias = 1
        m_dist_bias = 4
        neighbor_bias = 2
        score = exit_bias*((wrld.width()+wrld.height()) - exit_dist) + m_dist_bias*(m_dist_penalty/num_monsters) + neighbor_bias*(len(neighbors)-len(not_free_spaces)) #.4*(30 - exit) + .5*(m_dist_penalty/num_monsters) + .1*(1/(1+len(not_free_spaces)))
        score = score/(exit_bias*(wrld.width()+wrld.height()) + m_dist_bias*(12) + neighbor_bias*(8))
        # variant 2 .7*(30 - exit_dist) + 2.3*(m_dist_penalty/num_monsters) + .4*(len(neighbors)-len(not_free_spaces))
        # v2, v3, v4 .7*(30 - exit_dist) + 2.3*(m_dist_penalty/num_monsters) + .9*(len(neighbors)-len(not_free_spaces))
        #print(score)
        return score'''