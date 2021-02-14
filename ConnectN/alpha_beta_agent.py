import math
import random

import agent


###########################
# Alpha-Beta Search Agent #
###########################

class AlphaBetaAgent(agent.Agent):
    """Agent that uses alpha-beta search"""

    # Class constructor.
    #
    # PARAM [string] name:      the name of this player
    # PARAM [int]    max_depth: the maximum search depth
    def __init__(self, name, max_depth):
        super().__init__(name)
        # Max search depth
        self.max_depth = max_depth
        self.table = dict()
        self.table_len = 10000
        self.hits = 0

    def isConnectable(self, brd):
        pass

    def terminalTest(self, brd):
        return brd.get_outcome() != 0

    def staticEval(self, brd):
        board = brd.copy().board
        for i in range(len(board)):
            board[i] = tuple(board[i])
        board = tuple(board)
        print(board)
        brd_hash = hash(board)
        print("Hash: ", hash(board))
        print("Length: ", len(self.table))
        print("Hits: ", self.hits)

        if hash(board) in self.table:
            value = self.table[brd_hash]
            self.hits += 1
            return value
        else:
            # magic stuff
            value = int(random.randrange(1000))
            self.table[brd_hash] = value
            if len(self.table) > self.table_len:
                iterator = iter(self.table)
                del self.table[next(iterator)]
            return value

    def minimax(self, brd, col, depth, alpha, beta, maximizingPlayer): # First
        if depth == 0 or self.terminalTest(brd):
            return self.staticEval(brd), col

        if maximizingPlayer:
            maxEval = -99999999
            best_action = col
            for child in self.get_successors(brd):
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                #maxEval = max(maxEval, evalu)
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = child[1]

                alpha = max(alpha, evalu)
                if beta <= alpha:
                    print("Pruned")
                    break
            return maxEval, best_action

        else:
            minEval = 99999999
            best_action = col
            for child in self.get_successors(brd):
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                #minEval = min(minEval, evalu)
                if min(minEval, evalu) < minEval:
                    minEval = evalu
                    best_action = child[1]

                beta = min(beta, evalu)
                if beta <= alpha:
                    print("Pruned")
                    break

            return minEval, best_action

    # Pick a column.
    #
    # PARAM [board.Board] brd: the current board state
    # RETURN [int]: the column where the token must be added
    #
    # NOTE: make sure the column is legal, or you'll lose the game.
    def go(self, brd):
        """Search for the best move (choice of column for the token)"""
        # Your code here
        '''board = brd.copy().board  # make copy of input board
        table_len = 3
        for i in range(len(board)):
            board[i] = tuple(board[i])
        print(tuple(board))
        brd_hash = hash(tuple(board))
        print("Hash: ", hash(tuple(board)))
        self.table[brd_hash] = brd_hash
        if len(self.table) > table_len:
            iterator = iter(self.table)
            del self.table[next(iterator)]

        for i in range(4,1,-1):
            print(i)
        moves = [[],[],[]]
        for i in range(4,1, -1): # from 4 to 2 starting at 4
            brd.n = i
            counter = 0
            new = []
            for x in range(brd.w):
                for y in range(brd.h):
                    if (brd.board[y][x] != 0) and brd.is_any_line_at(x,y):
                        new.append([x,y])
            moves[4-i] = new
        print(moves)'''

        value, move = self.minimax(brd, -1, 4, -99999999, 99999999, True)
        print("Results!: ", value, move)

        if 3 in brd.free_cols():
            return 3
        else:
            return move

    # Get the successors of the given board.
    #
    # PARAM [board.Board] brd: the board state
    # RETURN [list of (board.Board, int)]: a list of the successor boards,
    #                                      along with the column where the last
    #                                      token was added in it
    def get_successors(self, brd):
        """Returns the reachable boards from the given board brd. The return value is a tuple (new board state, column number where last token was added)."""
        # Get possible actions
        freecols = brd.free_cols()
        # Are there legal actions left?
        if not freecols:
            return []
        # Make a list of the new boards along with the corresponding actions
        succ = []
        for col in freecols:
            # Clone the original board
            nb = brd.copy()
            # Add a token to the new board
            # (This internally changes nb.player, check the method definition!)
            nb.add_token(col)
            # Add board to list of successors
            succ.append((nb, col))
        return succ
