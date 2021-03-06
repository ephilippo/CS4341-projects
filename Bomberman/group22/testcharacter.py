# This is necessary to find the main code
import sys

sys.path.insert(0, '../bomberman')
# Import necessary stuff
from entity import CharacterEntity
from colorama import Fore, Back

# my imports
from queue import PriorityQueue
import math


class TestCharacter(CharacterEntity):

    def do(self, wrld):
        path = self.aStar(wrld.exitcell[0], wrld.exitcell[1], wrld)
        print(path)

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
            if (current == goal):
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

        return path.reverse()

    def heuristic(self, goal, next):
        return math.sqrt((goal[0] - next[0]) ** 2 + (goal[1] - next[1]) ** 2)

    def cost(self, next, wrld):
        if (wrld.wall_at(next[0], next[1])):
            return 10
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
