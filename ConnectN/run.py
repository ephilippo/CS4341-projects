import random
import game
import agent
import alpha_beta_agent as aba
import time

# Set random seed for reproducibility


#
# Random vs. Random
#
# g = game.Game(7, # width
#               6, # height
#               4, # tokens in a row to win
#               agent.RandomAgent("random1"),       # player 1
#               agent.RandomAgent("random2"))       # player 2

#
# Human vs. Random
#
# g = game.Game(7, # width
#               6, # height
#               4, # tokens in a row to win
#               agent.InteractiveAgent("human"),    # player 1
#               agent.RandomAgent("random"))        # player 2

#
# Random vs. AlphaBeta
#
# g = game.Game(7, # width
#               6, # height
#               4, # tokens in a row to win
#               agent.RandomAgent("random"),        # player 1
#               aba.AlphaBetaAgent("alphabeta", 6)) # player 2

#
# Human vs. AlphaBeta
#
# g = game.Game(7, # width
#               6, # height
#               4, # tokens in a row to win
#               agent.InteractiveAgent("human"),    # player 1
#               aba.AlphaBetaAgent("alphabeta", 5)) # player 2

#
# Human vs. Human
#
# g = game.Game(7, # width
#               6, # height
#               4, # tokens in a row to win
#               agent.InteractiveAgent("human1"),   # player 1
#               agent.InteractiveAgent("human2"))   # player 2
# outcome = g.go()
# Execute the game
from numpy import mean

t0 = time.time()
wins = []
losses = []
ties = []
times = []
av_times = []
games = 0
win = 0
loss = 0
tie = 0
subtimes = []
for i in range(0, 100):
    print(f"Depth: 5 Game: {i} {round((games/1000)*100, ndigits=1)}% Complete", end="\r")
    tx = time.time()
    g = game.Game(7, # width
                  6, # height
                  4, # tokens in a row to win
                  agent.RandomAgent("random"),
                  aba.AlphaBetaAgent("alphabeta", 5))
    outcome = g.go()
    if outcome == 2:
        win += 1
    elif outcome == 1:
        loss += 1
    else:
        tie += 1
    ts = time.time()
    subtimes.append(ts-tx)
    games += 1

t1 = time.time()
total = t1-t0
wins.append(win)
losses.append(loss)
ties.append(tie)
times.append(total)
av_times.append(mean(subtimes))

print("Results for depth-:", 5, "Wins: ", wins[0], " Losses: ", losses[0], " Ties: ", ties[0], "Win pct: ", (wins[0]/(wins[0]+losses[0]))*100, "Elapsed Time: ", times[0], "Average Time: ", av_times[0])
