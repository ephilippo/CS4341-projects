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
#               aba.AlphaBetaAgent("alphabeta", 4)) # player 2

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
random.seed(6)
games = 0
for j in range(2, 7):
    win = 0
    loss = 0
    tie = 0
    t0 = time.time()
    subtimes = []
    for i in range(0, 100):
        print(f"Depth: {j} Game: {i} {round((games/300)*100, ndigits=1)}% Complete", end="\r")
        tx = time.time()
        g = game.Game(7, # width
                      6, # height
                      4, # tokens in a row to win
                      agent.RandomAgent("random"),        # player 1
                      aba.AlphaBetaAgent("alphabeta", j)) # player 2
        outcome = g.go()
        if outcome == 2:
            win += 1
        elif outcome == 1:
            loss += 1
        else:
            tie += 1
        ts = time.time()
        subtimes.append(ts-tx)
        games +=1

    t1 = time.time()
    total = t1-t0
    wins.append(win)
    losses.append(loss)
    ties.append(tie)
    times.append(total)
    av_times.append(mean(subtimes))

for i in range(0, 5):
    print("Results for depth-:", i+2, "Wins: ", wins[i], " Losses: ", losses[i], " Ties: ", ties[i], "Win pct: ", (wins[i]/(wins[i]+losses[i]))*100, "Elapsed Time: ", times[i], "Average Time: ", av_times[i])
