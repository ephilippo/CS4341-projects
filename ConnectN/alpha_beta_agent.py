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

    def heuristic(self, brd, col, maximizingPlayer):
        #print("calling heuristic...")

        def countLineValue(x, y, dx, dy):
            """Return True if a line of identical tokens exists starting at (x,y) in direction (dx,dy)"""
            # Avoid out-of-bounds errors
            '''if ((x + (self.n-1) * dx >= self.w) or
                    (y + (self.n-1) * dy < 0) or (y + (self.n-1) * dy >= self.h)):
                return False'''
            # Get token at (x,y)

            t = brd.board[y][x]
            # Go through elements
            myPieces = 1
            holes = 0
            savedrow = 0
            for i in range(1, brd.n):
                inarow = 0
                if ((y + i * dy) >= brd.h) or ((x + i * dx) >= brd.w):
                    break
                elif brd.board[y + i*dy][x + i*dx] != t and brd.board[y + i*dy][x + i*dx] != 0:
                    break
                elif brd.board[y + i*dy][x + i*dx] == t:
                    inarow += 1
                    myPieces += 1
                elif brd.board[y + i*dy][x + i*dx] == 0:
                    if inarow > savedrow:
                        savedrow = inarow
                    inarow = 0
                    holes += 1

            for i in range(1, brd.n):
                inarow = 0
                if ((y - i * dy) < 0) or ((x - i * dx) < 0):
                    break
                elif brd.board[y - i*dy][x - i*dx] != t and brd.board[y - i*dy][x - i*dx] != 0:
                    break
                elif brd.board[y - i*dy][x - i*dx] == t:
                    inarow += 1
                    myPieces += 1
                elif brd.board[y - i*dy][x - i*dx] == 0:
                    if inarow > savedrow:
                        savedrow += inarow
                    inarow = 0
                    holes += 1

            value = 0
            if myPieces < brd.n and holes == 0:
                return 10
            elif holes + myPieces < brd.n:
                return 10
            elif holes == 0:
                value = 100**brd.n
            elif abs(dx) != abs(dy):
                value = 10**(myPieces*savedrow)
            else:
                value = 5**(myPieces*savedrow)

            return value

        y = 0
        for i in range(brd.h-1, -1, -1):
            if brd.board[i][col] != 0:
                y = i

        horiz_value = countLineValue(col, y, 1, 0)
        vert_value = countLineValue(col, y, 0, 1)
        diag_pos_value = countLineValue(col, y, 1, 1)
        diag_neg_value = countLineValue(col, y, 1, -1)

        value = horiz_value + vert_value + diag_pos_value + diag_neg_value

        #print("Calculated Value: ", value)
        if maximizingPlayer:
            return value
        else:
            return -1*value


    def staticEval(self, brd, col, maximizingPlayer):
        board = brd.copy().board
        '''for i in range(len(board)):
            board[i] = tuple(board[i])
        board = tuple(board)
        #print(board)
        brd_hash = hash(board)
        print("Hash: ", hash(board))
        print("Length: ", len(self.table))
        print("Hits: ", self.hits)'''

        '''if hash(board) in self.table:
            value = self.table[brd_hash]
            self.hits += 1
            #print("Value: ", value)
            if maximizingPlayer:
                return value
            else:
                return -1*value
        else:'''
        # magic stuff
        value = self.heuristic(brd, col, maximizingPlayer)
        '''self.table[brd_hash] = value
        if len(self.table) > self.table_len:
            iterator = iter(self.table)
            del self.table[next(iterator)]'''
        #print("Value: ", value)
        return value

    def minimax(self, brd, col, depth, alpha, beta, maximizingPlayer): # First
        if depth == 0 or self.terminalTest(brd):
            return self.staticEval(brd, col, maximizingPlayer), col

        if maximizingPlayer:
            maxEval = -99999999
            best_action = col
            for child in self.get_successors(brd):
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                #maxEval = max(maxEval, evalu)
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = column

                alpha = max(alpha, evalu)
                if beta <= alpha:
                    #print("Pruned")
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
                    best_action = column

                beta = min(beta, evalu)
                if beta <= alpha:
                    #print("Pruned")
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
        value, move = self.minimax(brd, -1, self.max_depth, -99999999, 99999999, True)
        print("Results!: ", value, move)

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
