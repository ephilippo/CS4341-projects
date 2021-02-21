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

    # Terminal Test
    #
    # PARAM [board.Board]
    #
    # RETURN [bool]: whether there is a 4 in a row on the current board state
    def terminalTest(self, brd):
        return brd.get_outcome() != 0

    # Nearest Pieces
    #
    # PARAM [board.Board] brd: the current board state
    # PARAM [int] x: the column of the current placed piece
    # PARAM [int] y: the row of the current placed piece
    # PARAM [int] dx: direction in the x
    # PARAM [int] dy: direction in the y
    #
    # RETURN [list.List]: the list of nearest 'n' pieces in the given direction
    def nearestPieces(self, brd, x, y, dx, dy):
        t = brd.board[y][x] # getting the value of the current players piece
        pieces = []  # list of nearest pieces

        # Helper Function: Check if move is in Bounds
        #
        # PARAM [int] x: the current column of the player
        # PARAM [int] y: the current row of the player
        # PARAM [int] i: a multiple factor to check how far away you need to check the bounds
        #
        # Return [bool]: whether the next move is in bounds
        def inBounds(x, y, i):
            return not(((y + i * dy) >= brd.h) or ((y + i * dy) < 0) or ((x + i * dx) >= brd.w) or ((x + i * dx) < 0))

        if inBounds(x, y, 1): # if the first step in the given direction is in bounds
            if brd.board[y + 1*dy][x + 1*dx] != t and brd.board[y + 1*dy][x + 1*dx] != 0:
                # if the first step is an opponent's piece,
                # find out how many opponent's pieces our move is blocking
                pieces.append(brd.board[y + 1*dy][x + 1*dx])
                i = 2
                while (i < brd.n) and (inBounds(x, y, i)):
                    if brd.board[y + i*dy][x + i*dx] != t and brd.board[y + i*dy][x + i*dx] != 0:
                        pieces.append(brd.board[y + i*dy][x + i*dx])
                        i +=1
                    else:
                        # we break because we are uninterested in what is past an opponent's piece
                        break
            else:
                # if the first step is not an opponent,
                # find what empty spaces and/or our pieces are nearby
                i = 1
                while (i < brd.n) and (inBounds(x, y, i)):
                    if brd.board[y + i*dy][x + i*dx] != t and brd.board[y + i*dy][x + i*dx] != 0:
                        # we break because we are uninterested in what is past an opponent's piece,
                        # we don't append it either due to this reasoning
                        break
                    elif brd.board[y + i*dy][x + i*dx] == t:
                        pieces.append(brd.board[y + i*dy][x + i*dx])
                    elif brd.board[y + i*dy][x + i*dx] == 0:
                        pieces.append(brd.board[y + i*dy][x + i*dx])
                    i +=1
                i = 0

        return pieces

    # Heuristic Function
    #
    # PARAM [board.Board] brd: the current board state
    # PARAM [list.List] pieces_forward: the nearest 'n' pieces to the right of the placed piece
    # PARAM [list.List] pieces_backward: the nearest 'n' pieces to the left of the placed piece
    # PARAM [int] t: the current players piece
    #
    # RETURN [int]: the evaluation of a terminal node
    def heuristic(self, brd, pieces_forward, pieces_backward, t):
        pieces_backward.reverse()
        #pieces = pieces_backward + [t] + pieces_forward
        pieces = pieces_backward + pieces_forward
        # the above two lines create a single list containing
        # all the pieces near the placed piece in the order
        # they naturally occur: including the placed piece
        heuristic_value = 0
        sets = []  # list of all the subsets of length 'n' containing the placed piece
        for i in range((len(pieces)-(brd.n-1))):
            temp = []
            for j in range(brd.n):
                temp.append(pieces[i+j])
            sets.append(temp)

        subset_num = 0
        for subset in sets:
            subset.reverse()  # reversing the order to make the subset easier to parse
            i = subset_num
            while i < len(subset):
                # the farther away a piece is from the placed piece the less value it holds
                if subset[subset_num] == t and t == 2:
                    heuristic_value += subset[subset_num] * 10**(10-2*i)
                elif subset[subset_num] == t and t == 1:
                    heuristic_value += (2*subset[subset_num]) * 10**(10-2*i)
                i += 1
            subset_num += 1
        return heuristic_value

    # Static Evaluation
    #
    # PARAM [board.Board] brd: the current board state
    # PARAM [int] col: the column for which the most recent piece was placed
    #
    # RETURN [int]: the evaluation of a terminal node
    def staticEval(self, brd, col):
        y = 0
        for i in range(brd.h-1, -1, -1):  # finding which row the placed piece fell to
            if brd.board[i][col] != 0:
                y = i
                break
        # combining together the heuristic values for each direction around the placed piece
        horizontal_val = self.heuristic(brd, self.nearestPieces(brd, col, y, 1, 0), self.nearestPieces(brd, col, y, -1, 0), brd.board[y][col])
        vertical_val = self.heuristic(brd, [], self.nearestPieces(brd, col, y, 0, -1), brd.board[y][col])
        diagonal_up_val = self.heuristic(brd, self.nearestPieces(brd, col, y, 1, 1), self.nearestPieces(brd, col, y, -1, -1), brd.board[y][col])
        diagonal_down_val = self.heuristic(brd, self.nearestPieces(brd, col, y, 1, -1), self.nearestPieces(brd, col, y, -1, 1), brd.board[y][col])
        value = horizontal_val + vertical_val + diagonal_up_val + diagonal_down_val
        return value

    # Minimax Algorithm
    #
    # PARAM [board.Board] brd: the current board state
    # PARAM [int] col: the column for which the most recent piece was placed
    # PARAM [int] depth: the maximum depth for minimax to search
    # PARAM [int] alpha: the largest minimized number, for alpha-beta pruning purposes
    # PARAM [int] beta: the smallest maximized number, for alpha-beta pruning purposes
    # PARAM [bool] beta: to determine whether the current node is a maximizing or minimizing node
    #
    # RETURN [int]: the value related to this node
    # RETURN [int]: the move that is predicted to produce the best value for the player
    def minimax(self, brd, col, depth, alpha, beta, maximizingPlayer): # First
        if depth == 0 or self.terminalTest(brd):
            return self.staticEval(brd, col), col
        if maximizingPlayer:
            maxEval = -10**15
            best_action = col
            for child in self.get_successors(brd):  # getting the children of the current node
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = child[1]  # the best move will have the best value
                alpha = max(alpha, evalu)  # alpha-beta pruning
                if beta <= alpha:
                    break
            return maxEval, best_action
        else:
            minEval = 10**15
            best_action = col
            for child in self.get_successors(brd):  # getting the children of the current node
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                if min(minEval, evalu) < minEval:
                    minEval = evalu
                    best_action = child[1]  # the best move will have the best value
                beta = min(beta, evalu)  # alpha-beta pruning
                if beta <= alpha:
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
        value, move = self.minimax(brd, -1, self.max_depth, -10**15, 10**15, True) # calling minimax function
        return move

    # Get the successors of the given board.
    #
    # PARAM [board.Board] brd: the board state
    # RETURN [list of (board.Board, int)]: a list of the successor boards,
    #                                      along with the column where the last
    #                                      token was added in it
    def get_successors(self, brd):
        """Returns the reachable boards from the given board brd.
        The return value is a tuple (new board state, column number where last token was added)."""
        # Get possible actions
        freecols = brd.free_cols()
        # Are there legal actions left?
        if not freecols:
            return []
        # Make a list of the new boards along with the corresponding actions
        succ = []
        length = len(freecols)
        mid = length//2
        fc_1 = freecols[:mid]
        fc_2 = freecols[mid:]
        fc_1.reverse()
        midfirst = []
        fc_1c = 0
        fc_2c = 0
        for i in range(length):
            if i%2 == 1:
                midfirst.append(fc_1[fc_1c])
                fc_1c += 1
            if i%2 == 0:
                midfirst.append(fc_2[fc_2c])
                fc_2c += 1
        for col in midfirst:
            # Clone the original board
            nb = brd.copy()
            # Add a token to the new board
            # (This internally changes nb.player, check the method definition!)
            nb.add_token(col)
            # Add board to list of successors
            succ.append((nb, col))
        return succ
