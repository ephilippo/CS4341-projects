# This is necessary to find the main code
import random
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back, Style
from monsters.selfpreserving_monster import SelfPreservingMonster

# my imports
from queue import PriorityQueue
import math

class TestCharacter(CharacterEntity):

    def __init__(self, name, avatar, x, y):
        CharacterEntity.__init__(self, name, avatar, x, y)
        self.times = 0
        self.clock = 0
        self.explosion = []
        self.placedBomb = False
        self.exclude = False

    def explosionPath(self,wrld,bombX,bombY):
        expPath = [(bombX,bombY)]
        count = 1
        while count <= 4:
            if(bombX-count >= 0): expPath.append((bombX-count,bombY))
            if(bombX+count <= wrld.width()): expPath.append((bombX+count,bombY))
            count +=1
        count = 1
        while count <= 4:
            if(bombY-count >= 0): expPath.append((bombX,bombY-count))
            if(bombY+count <= wrld.height()): expPath.append((bombX,bombY+count))
            count +=1
        return expPath

    def validMove(self, wrld, x, y, explosion):
        if x-1 >= 0 and ((x-1,y) not in explosion) and not wrld.wall_at(x-1,y): return x-1, y
        if x+1 < wrld.width() and ((x+1,y) not in explosion) and not wrld.wall_at(x+1,y): return x+1, y
        if y-1 >= 0 and ((x,y-1) not in explosion) and not wrld.wall_at(x,y-1): return x, y-1
        if y+1 < wrld.height() and ((x,y+1) not in explosion) and not wrld.wall_at(x,y+1): return x, y+1

        if x-1 >= 0 and y -1 >= 0 and ((x-1,y-1) not in explosion) and not wrld.wall_at(x-1,y-1): return x-1, y-1
        if x-1 >= 0 and y +1 < wrld.height() and ((x-1,y+1) not in explosion) and not wrld.wall_at(x-1,y+1): return x-1, y+1
        if x+1 < wrld.width() and y -1 >= 0 and ((x+1,y-1) not in explosion) and not wrld.wall_at(x+1,y-1): return x+1, y-1
        if x+1 < wrld.width() and y +1 <= wrld.height() and ((x+1,y+1) not in explosion) and not wrld.wall_at(x+1,y+1): return x+1,y+1
        if ((x, y) not in explosion) and not wrld.wall_at(x+1,y+1): return x,y


    def do(self, wrld):
        x = wrld.me(self).x
        y = wrld.me(self).y
        char_exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)

        if self.monstersInPlay(wrld) == 0:
            if len(self.wallsInWay(wrld, set(char_exit_path))) == 0:
                self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
            else:
                if(not self.bombInPlay(wrld) and not self.explosionInPlay(wrld)):
                    if(wrld.wall_at(char_exit_path[1][0], char_exit_path[1][1])):
                        self.place_bomb()
                        self.explosion = self.explosionPath(wrld, x, y)
                        self.placedBomb = True
                        self.move(1, -1)
                    else:
                        self.move(char_exit_path[1][0]-x, char_exit_path[1][1]-y)
                else:
                    if self.bombInPlay(wrld):
                        bombs = list(wrld.bombs.values())
                        for bomb in bombs:
                            if bomb.timer == 1:
                                self.explosion = self.explosionPath(wrld, bomb.x, bomb.y)
                    if self.explosion:
                        safe_x, safe_y = self.validMove(wrld, x, y, self.explosion)
                        self.move(safe_x-x, safe_y-y)
                        if not self.bombInPlay(wrld) and not self.explosionInPlay(wrld):
                            self.explosion = []

        else:
            monsters = list(wrld.monsters.values())
            within_range = []
            past_all = True
            past_all_2 = True
            minimax = False
            expectimax = False
            chamber1 = False
            chamber2 = False
            chamber3 = False
            chamber4 = False
            chamber1_barrier = []
            chamber2_barrier = []
            chamber3_barrier = []
            chamber4_barrier = []
            for i in range(wrld.width()):
                chamber1_barrier.append((i, 3))
                chamber2_barrier.append((i, 7))
                chamber3_barrier.append((i, 11))
                chamber4_barrier.append((i, 15))
            for val in chamber1_barrier:
                if not wrld.wall_at(val[0], val[1]):
                    chamber1_barrier.remove((val[0], val[1]))
            for val in chamber2_barrier:
                if not wrld.wall_at(val[0], val[1]):
                    chamber2_barrier.remove((val[0], val[1]))
            for val in chamber3_barrier:
                if not wrld.wall_at(val[0], val[1]):
                    chamber3_barrier.remove((val[0], val[1]))
            for val in chamber4_barrier:
                if not wrld.wall_at(val[0], val[1]):
                    chamber4_barrier.remove((val[0], val[1]))

            for mon_idx in monsters:
                for mon in mon_idx:
                    mon_x = mon.x
                    mon_y = mon.y
                    mon_exit_path = self.aStar(mon_x, mon_y, wrld.exitcell[0], wrld.exitcell[1], wrld)
                    char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)

                    shortest = (100, [])
                    counter = 0
                    for val in char_exit_path:
                        val_path_to_mon = self.aStar(val[0], val[1], mon_x, mon_y, wrld)
                        counter += 1
                        if min(len(val_path_to_mon), shortest[0]) < shortest[0] or (len(val_path_to_mon) == shortest[0]):
                            shortest = (len(val_path_to_mon), val_path_to_mon, val, counter)

                    if mon_y < 7 and mon_y > 3:
                        chamber1 = True
                    if mon_y > 7 and mon_y < 11:
                        chamber2 = True
                    if mon_y > 11 and mon_y < 15:
                        chamber3 = True
                    if mon_y > 15:
                        chamber4 = True

                    if y < (mon_y + 2):
                        past_all = False
                    if len(char_exit_path) >= (len(mon_exit_path)-1) or ((y < mon_y) and (counter >= (len(shortest[1])-2))):
                        past_all_2 = False
                    if len(char_to_mon) <= 6:
                        within_range.append(mon)
                        if len(char_to_mon) <= 4:
                            minimax = True
                        else:
                            expectimax = True

            dx = char_exit_path[1][0]-x
            dy = char_exit_path[1][1]-y
            walls = self.wallsInWay(wrld, set(char_exit_path))
            if walls and (chamber1 or chamber2 or chamber3 or chamber4) and not(self.bombInPlay(wrld)):
                self.exclude = True
                monsters = list(wrld.monsters.values())
                flip = False
                path = self.aStar(x, y, 7, 4*self.times, wrld)
                for mon_idx in monsters:
                    for mon in mon_idx:
                        mon_x = mon.x
                        mon_y = mon.y
                        '''char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)
                        if len(char_to_mon) <= 8:
                            caught = True'''
                        if mon.x == 7:
                            flip = True
                            self.times = 0
                        if mon.x == 0:
                            flip = False
                if flip:
                    path = self.aStar(x, y, 0, 4*self.times, wrld)
                    if (x, y) != (0, 4*self.times): # and not caught
                        self.move(path[1][0]-x, path[1][1]-y)
                    elif (x, y) == (0, 4*self.times):
                        self.place_bomb()
                        self.move(0, -1)
                        self.times += 1
                    if self.times == 4:
                        self.times = 0
                else:

                    if (x, y) != (7, 4*self.times): # and not caught
                        self.move(path[1][0]-x, path[1][1]-y)
                    elif (x, y) == (7, 4*self.times):
                        self.place_bomb()
                        self.move(0, -1)
                        self.times += 1
                    if self.times == 4:
                        self.times = 0
                    '''if caught:
                        self.place_bomb()
                        self.move(-1, 0)'''

            if (not walls) and self.exclude:
                monsters = list(wrld.monsters.values())
                caught = False
                for mon_idx in monsters:
                    for mon in mon_idx:
                        mon_x = mon.x
                        mon_y = mon.y
                        char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)
                        if len(char_to_mon) <= 8:
                            caught = True
                '''if caught:
                    self.place_bomb()
                    self.move(-1, 0)
                else:'''
                self.move(dx, dy)
            if self.bombInPlay(wrld):
                bombs = list(wrld.bombs.values())
                for bomb in bombs:
                    if bomb.timer == 1:
                        self.explosion = self.explosionPath(wrld, bomb.x, bomb.y)
                monsters = list(wrld.monsters.values())
                caught = False
                for mon_idx in monsters:
                    for mon in mon_idx:
                        mon_x = mon.x
                        mon_y = mon.y
                        char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)
                        if len(char_to_mon) <= 8:
                            caught = True
                '''if caught:
                    self.place_bomb()
                    self.move(-1, 0)
                else:
                    self.move(dx, dy)'''
            if self.explosion:
                safe_x, safe_y = self.validMove(wrld, x, y, self.explosion)
                self.move(safe_x-x, safe_y-y)
                if not self.bombInPlay(wrld) and not self.explosionInPlay(wrld):
                    self.explosion = []
            elif (past_all or past_all_2) and not self.exclude:
                self.move(dx, dy)
            elif minimax and not self.exclude:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 4, True, alpha, beta, True, within_range)
                self.move(move[0], move[1])
            elif expectimax and not self.exclude:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 4, True, alpha, beta, False, within_range)
                self.move(move[0], move[1])
            elif not self.exclude:
                self.move(dx, dy)

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
                score = -10000
                found = True
            if e.tpe == e.CHARACTER_KILLED_BY_MONSTER:
                score = -10000
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
            monsters = list(wrld.monsters.values())
            safe = True
            for mon_idx in monsters:
                for mon in mon_idx:
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
                    if len(mon_path) < safe_dist:
                        safe = False

                if safe:
                    score += 120
                    score -= len(char_exit_path)
                else:
                    score -= 300
                    score += len(mon_path)*3


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
            #monsters = list(wrld.monsters.values())
            for m in monsters:
                m.move(0, 0)
            char_moves = self.neighbors((x, y), wrld)
            #print(char_moves)
            for val in char_moves:
                if (not wrld.wall_at(val[0], val[1])) and (not wrld.explosion_at(val[0], val[1])):
                    wrld.me(self).move(val[0]-x, val[1]-y)
                    succ.append((wrld.next()[0], val[0]-x, val[1]-y))
        else:
            wrld.me(self).move(0, 0)
            #monsters = list(wrld.monsters.values())
            for m in monsters:
                mon_path = self.aStar(x, y, m.x, m.y, wrld)
                if len(mon_path) > 4:
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
                    standin = SelfPreservingMonster("standin", "Z", m.x, m.y, 3)
                    (found, dx, dy) = standin.look_for_character(wrld)
                    if found and not standin.must_change_direction(wrld):
                        m.move(dx, dy)
                    # If I'm idle or must change direction, change direction
                    elif ((m.dx == 0 and m.dy == 0) or standin.must_change_direction(wrld)):
                        # Get list of safe moves
                        safe = standin.look_for_empty_cell(wrld)
                        if safe:
                            move = random.choice(safe)
                            m.move(move[0], move[1])
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
                    #print("pruned", beta, alpha, maxEval, maximizingPlayer)
                    break
                #print(depth, evalu, child[1], child[2], maximizingPlayer, maxEval, best_action[0], best_action[1])
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
                #print(depth, evalu, child[1], child[2], maximizingPlayer)
                if mini:
                    if min(minEval, evalu) < minEval:
                        minEval = evalu  # the best move will have the best value
                    beta = min(beta, evalu)  # alpha-beta pruning
                    if beta <= alpha:
                        #print("pruned", beta, alpha, minEval, maximizingPlayer)
                        break
                else:
                    sum_vals += evalu
                    length += 1
                    beta = 1000*(len(succ)-length) + sum_vals # alpha-beta pruning
                    if beta <= alpha:
                        #print("pruned", beta, alpha, len(succ), length, sum_vals, maximizingPlayer)
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
                    new_cost = cost_so_far[current] + self.cost(current, next, wrld)
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
        return max(abs(goal[0] - next[0]), abs(goal[1] - next[1]))

    def cost(self, current, next, wrld):
        dx = next[0]-current[0]
        dy = next[1]-current[1]
        if wrld.wall_at(next[0], next[1]):
            cost = 10
        elif dx*dy:
            cost = math.sqrt(2)
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
