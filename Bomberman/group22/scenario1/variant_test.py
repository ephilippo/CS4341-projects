# This is necessary to find the main code
import sys
sys.path.insert(0, '../../bomberman')
sys.path.insert(1, '..')

# Import necessary stuff
import random
from game import Game
from monsters.selfpreserving_monster import SelfPreservingMonster
from monsters.stupid_monster import StupidMonster

# TODO This is your code!
sys.path.insert(1, '../groupNN')
from testcharacter import TestCharacter

# Create the game
#23452352
#76457563
random.seed(23452352) # TODO Change this if you want different random choices
g = Game.fromfile('map.txt')
'''g.add_monster(StupidMonster("stupid", # name
                            "S",      # avatar
                            6, 12,     # position
                            ))

g.add_monster(SelfPreservingMonster("aggressive", # name
                                    "A",          # avatar
                                    3, 6,        # position
                                    2             # detection range
                                    ))'''

# TODO Add your character
g.add_character(TestCharacter("me", # name
                              "C",  # avatar
                              2, 8  # position
))

# Run!
g.go()
