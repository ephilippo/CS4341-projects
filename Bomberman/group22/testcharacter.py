# This is necessary to find the main code
import random
import sys

from Bomberman.bomberman.monsters.selfpreserving_monster import SelfPreservingMonster

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
        self.wait = 0
        self.times = 0

    def explosionsInPlay(self, wrld):
        return len(wrld.explosions)

    def bombsInPlay(self, wrld):
        return len(wrld.bombs)

    def monstersInPlay(self, wrld):
        return len(wrld.monsters)

    def charsInPlay(self, wrld):
        return len(wrld.characters)

    def nonTrivialWorld(self, wrld):
        return self.monstersInPlay(wrld) and self.charsInPlay(wrld)

    def charDied(self, wrld):
        for e in wrld.events:
            if e.tpe == e.BOMB_HIT_CHARACTER:
                return True
            if e.tpe == e.CHARACTER_KILLED_BY_MONSTER:
                return True
        return False

    def monsterDied(self, wrld):
        for e in wrld.events:
            if e.tpe == e.BOMB_HIT_MONSTER:
                return True
        return False

    def charFoundExit(self, wrld):
        for e in wrld.events:
            if e.tpe == e.CHARACTER_FOUND_EXIT:
                return True
        return False

    def terminalTest(self, wrld):
        # if we are in the same place as a monster then we are dead
        if self.charDied(wrld) or self.charFoundExit(wrld):
            return True
        allMonsters = self.monsterDetector(wrld)
        _, nearMonsters = self.monstersNearMe(wrld, allMonsters, 6)
        if not nearMonsters:
            return True
        return False

    def getCharXnY(self, wrld):
        return wrld.me(self).x, wrld.me(self).y

    def genExitPath(self, wrld, x, y):
        exit_path = self.aStar(x, y, wrld.exitcell[0], wrld.exitcell[1], wrld)
        return exit_path

    def wallsInWay(self, wrld, path):
        wallsInWay = set()
        for i in range(wrld.width()):
            for j in range(wrld.height()):
                if wrld.grid[i][j] and ((i, j) in set(path)):
                    wallsInWay.add((i, j))
        return wallsInWay

    def monsterDetector(self, wrld):
        listOfMonsters = []
        if self.nonTrivialWorld(wrld):
            monsters = list(wrld.monsters.values())
            for mon_idx in monsters:
                for mon in mon_idx:
                    listOfMonsters.append(mon)
        return listOfMonsters

    def monstersNearMe(self, wrld, monsters, range):
        within_range = []
        inDanger = False
        if monsters:
            x, y = self.getCharXnY(wrld)
            for monster in monsters:
                char_to_mon = self.aStar(x, y, monster.x, monster.y, wrld)
                if not self.wallsInWay(wrld, set(char_to_mon)):
                    if len(char_to_mon) <= range:
                        within_range.append(monster)
                        if monster.name == "stupid":
                            danger = 4
                        else:
                            danger = 5
                        if len(char_to_mon) <= danger:
                            inDanger = True
        return inDanger, within_range

    def canEscape(self, wrld, allMonsters):
        canEscape = True
        if self.nonTrivialWorld(wrld):
            x, y = self.getCharXnY(wrld)
            char_exit_path = self.genExitPath(wrld, x, y)
            for mon in allMonsters:
                mon_x = mon.x
                mon_y = mon.y
                mon_exit_path = self.genExitPath(wrld, mon_x, mon_y)

                shortest = (100, [])
                moves_away = 0
                for val in char_exit_path:
                    val_path_to_mon = self.aStar(val[0], val[1], mon_x, mon_y, wrld)
                    moves_away += 1
                    if min(len(val_path_to_mon), shortest[0]) < shortest[0] or (len(val_path_to_mon) == shortest[0]):
                        shortest = (len(val_path_to_mon), val_path_to_mon, val, moves_away)

                not_strictly_past_mon = (y < (mon_y + 2))
                cant_out_run = len(char_exit_path) >= (len(mon_exit_path) - 1)
                cant_squeeze_by = ((y < mon_y) and (moves_away >= (len(shortest[1]) - 2)))

                if not_strictly_past_mon or cant_out_run or cant_squeeze_by:
                    canEscape = False
        return canEscape

    def do(self, wrld):
        x, y = self.getCharXnY(wrld)
        char_exit_path = self.genExitPath(wrld, x, y)
        dx = char_exit_path[1][0] - x
        dy = char_exit_path[1][1] - y
        '''path_set = set(char_exit_path)
        print(char_exit_path)
        self.printAStar(path_set, wrld)'''
        allMonsters = self.monsterDetector(wrld)
        if allMonsters:
            inDanger, nearMonsters = self.monstersNearMe(wrld, allMonsters, 6)
            if self.canEscape(wrld, allMonsters):
                self.move(dx, dy)
            elif inDanger:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 4, True, alpha, beta, minimax=True)
                self.move(move[0], move[1])
            elif nearMonsters:
                alpha = -10000
                beta = 1000
                _, move = self.expectimax(wrld, (0, 0), 4, True, alpha, beta, minimax=False)
                self.move(move[0], move[1])
            else:
                self.move(dx, dy)
        else:
            if len(self.wallsInWay(wrld, set(char_exit_path))) == 0:
                self.move(dx, dy)
            else:
                self.move(dx, dy)

    def expectimax(self, wrld, col, depth, maximizingPlayer, alpha, beta, minimax):
        if depth == 0 or self.terminalTest(wrld):
            return self.staticEval(wrld), col
        if maximizingPlayer:
            maxEval = -10 ** 15
            best_action = (0, 0)
            for child in self.getSuccessors(wrld, maximizingPlayer):  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth - 1, not (maximizingPlayer), alpha, beta, minimax)
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = (child[1], child[2])  # the best move will have the best value
                alpha = max(alpha, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    # print("pruned", beta, alpha, maxEval, maximizingPlayer)
                    break
                # print(depth, evalu, child[1], child[2], maximizingPlayer, maxEval, best_action[0], best_action[1])
            '''wrld.printit()
            print(maxEval)
            print('maximizing node')
            input("Press Enter to continue getting scores...")'''
            return maxEval, best_action
        else:
            best_action = (0, 0)
            succ = self.getSuccessors(wrld, maximizingPlayer)
            length = 0
            sum_vals = 0
            minEval = 1000
            for child in succ:  # getting the children of the current node
                evalu, column = self.expectimax(child[0], (child[1], child[2]), depth - 1, not (maximizingPlayer), alpha, beta, minimax)
                # print(depth, evalu, child[1], child[2], maximizingPlayer)
                if minimax:
                    if min(minEval, evalu) < minEval:
                        minEval = evalu  # the best move will have the best value
                    beta = min(beta, evalu)  # alpha-beta pruning
                    if beta <= alpha:
                        # print("pruned", beta, alpha, minEval, maximizingPlayer)
                        break
                else:
                    sum_vals += evalu
                    length += 1
                    beta = 1000 * (len(succ) - length) + sum_vals  # alpha-beta pruning
                    if beta <= alpha:
                        # print("pruned", beta, alpha, len(succ), length, sum_vals, maximizingPlayer)
                        break
            if minimax:
                return_val = minEval
            else:
                return_val = sum_vals / length
            return return_val, best_action

    def staticEval(self, wrld):
        if self.charDied(wrld):
            score = -10000
        elif self.charFoundExit(wrld):
            score = 1000
        elif self.nonTrivialWorld(wrld):
            x, y = self.getCharXnY(wrld)
            char_exit_path = self.genExitPath(wrld, x, y)
            col_value = [-100, -10, 1, 4, 4, 1, -10, -100]
            score = col_value[x]
            allMonsters = self.monsterDetector(wrld)
            inDanger, nearMonsters = self.monstersNearMe(wrld, allMonsters, 6)
            paths_to_monsters = []
            for mon in nearMonsters:
                mon_x = mon.x
                mon_y = mon.y
                char_to_mon = self.aStar(x, y, mon_x, mon_y, wrld)
                paths_to_monsters.append(char_to_mon)

            if len(paths_to_monsters) == 0:
                shorter_path = []
            elif len(paths_to_monsters) == 1:
                shorter_path = paths_to_monsters[0]
            elif len(paths_to_monsters[0]) <= len(paths_to_monsters[1]):
                shorter_path = paths_to_monsters[0]
            else:
                shorter_path = paths_to_monsters[1]

            if inDanger:
                score -= 300
                score += len(shorter_path) * 3
            else:
                score += 120
                score -= len(char_exit_path)
        elif self.monstersInPlay(wrld):
            score = -10000
        else:
            score = 0
        return score

    def getSuccessors(self, wrld, maximizingPlayer):
        """
        return all the possible next world states from the current world state
        """
        succ = []
        x, y = self.getCharXnY(wrld)
        allMonsters = self.monsterDetector(wrld)
        if maximizingPlayer:
            for m in allMonsters:
                m.move(0, 0)
            char_moves = self.neighbors((x, y), wrld)
            # print(char_moves)
            for val in char_moves:
                if not wrld.wall_at(val[0], val[1]):
                    wrld.me(self).move(val[0] - x, val[1] - y)
                    succ.append((wrld.next()[0], val[0] - x, val[1] - y))
        else:
            wrld.me(self).move(0, 0)
            for m in allMonsters:
                x, y = self.getCharXnY(wrld)
                mon_path = self.aStar(x, y, m.x, m.y, wrld)
                if len(mon_path) > 4:
                    # print("I'm with stupid")
                    for dx in [-1, 0, 1]:
                        # Avoid out-of-bound indexing
                        if (m.x + dx >= 0) and (m.x + dx < wrld.width()):
                            # Loop through delta y
                            for dy in [-1, 0, 1]:
                                if (dx != 0) or (dy != 0):
                                    # Avoid out-of-bound indexing
                                    if (m.y + dy >= 0) and (m.y + dy < wrld.height()):
                                        # if (not (dx == 0 and dy == 0)):
                                        if not wrld.wall_at(m.x + dx, m.y + dy):
                                            # Set move in wrld
                                            m.move(dx, dy)
                                            # Get new world
                                            succ.append((wrld.next()[0], dx, dy))
                else:
                    # print("I'm with angry")
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

    def aStar(self, x, y, goal_x, goal_y, wrld):
        # A* Implementation
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
            # print(tracker)
        path.reverse()
        return path

    def heuristic(self, goal, next):
        return max(abs(goal[0] - next[0]), abs(goal[1] - next[1]))

    def cost(self, current, next, wrld):
        dx = next[0] - current[0]
        dy = next[1] - current[1]
        if wrld.wall_at(next[0], next[1]):
            cost = 100
        elif dx * dy:
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
                if wrld.characters_at(x, y):
                    for c in wrld.characters_at(x, y):
                        sys.stdout.write(Back.GREEN + c.avatar)
                elif wrld.monsters_at(x, y):
                    for m in wrld.monsters_at(x, y):
                        sys.stdout.write(Back.BLUE + m.avatar)
                elif wrld.exit_at(x, y):
                    sys.stdout.write(Back.YELLOW + "#")
                elif wrld.bomb_at(x, y):
                    sys.stdout.write(Back.MAGENTA + "@")
                elif wrld.explosion_at(x, y):
                    sys.stdout.write(Fore.RED + "*")
                elif wrld.wall_at(x, y):
                    sys.stdout.write(Back.WHITE + " ")
                elif (x, y) in path_set:
                    sys.stdout.write(Back.RED + " ")
                else:
                    tile = False
                    for k, clist in wrld.characters.items():
                        for c in clist:
                            if c.tiles.get((x, y)):
                                sys.stdout.write(c.tiles[(x, y)] + ".")
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
