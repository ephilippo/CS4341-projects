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

    def checkLines(self, brd, x, y, dx, dy, N, t):
        # Avoid out-of-bounds errors
        print("Boardn during check line: ", brd.n)
        if ((x + (N-1) * dx >= brd.w) or
                (y + (N-1) * dy < 0) or (y + (N-1) * dy >= brd.h)):
            print("outta bounds...")
            return False
        # Go through elements
        count = 0
        holes = 0
        if ((x - dx < brd.w) or (y - dy >= 0) or (y - dy < brd.h) or (x - dx >= 0)):
            if brd.board[y - dy][x - dx] == 0:
                holes += 1

        for i in range(1, N):
            print("checklines", i)
            if not(brd.board[y + i*dy][x + i*dx] == t or brd.board[y + i*dy][x + i*dx] == 0):
                print("opponent piece")
                break
            elif brd.board[y + i*dy][x + i*dx] == t:
                print("mypiece")
                count += 1
            else:
                holes += 1
        if (count == brd.n-1) and ((count + holes >= N) or (brd.n == N)):
            print(count, holes, N)
            print(brd.n, " in a row!", x, y, dx, dy)
            return True
        else:
            print(count, holes, N)
            print(brd.n, " too many holes", x, y, dx, dy)
            return False

    def anyCheckLines(self, brd, x, y, N, t):
        print("Boardn pre checkline: ", brd.n)
        count = 0
        if self.checkLines(brd, x, y, 1, 0, N, t):
            count += 1
        if self.checkLines(brd, x, y, 0, 1, N, t):
            count += 1
        if self.checkLines(brd, x, y, 1, 1, N, t):
            count += 1
        if self.checkLines(brd, x, y, 1, -1, N, t):
            count += 1
        print("count: ", count)
        return count

    def heuristic_2(self, brd, col):
        board = brd.copy()
        vert = 0
        for i in range(board.h-1, -1, -1):
            if brd.board[i][col] != 0:
                vert = i
                print("inserted column and y: ", vert)
                break
        t = board.board[vert][col]
        N = brd.n
        myInaRows = []
        oppInaRows = []
        for i in range(N-1):
            print("heuristic_i", i)
            board.n = N - i
            print("Boardn heur: ", board.n)
            mine = 0
            opp = 0
            for x in range(board.w):
                for y in range(board.h):
                    if board.board[y][x] != 0:
                        count = self.anyCheckLines(board, x, y, N, board.board[y][x])
                        print("Boardn after check: ", board.n)
                        if count > 0:
                            #print("Here")
                            if board.board[y][x] == t:
                                print("adding mine")
                                mine += count
                            else:
                                print("adding opp")
                                opp += count
            myInaRows.append(mine)
            oppInaRows.append(opp)
        print("mine: ", myInaRows)
        print("opp: ", oppInaRows)
        total = []
        for i in range(N-1):
            total.append(myInaRows[i]*((10-(N-1))**(N-1)) - oppInaRows[i]*((10-(N-1))**(N-1)))
        suma = 0
        for i in range(N-1):
            suma += total[i]
        print("suma: ", suma)
        return suma


    def heuristic(self, brd, col, maximizingPlayer):
        #print("calling heuristic...")

        def countDirectionPieces(x, y, dx, dy):
            """Return True if a line of identical tokens exists starting at (x,y) in direction (dx,dy)"""
            # Get token at (x,y)
            t = brd.board[y][x]
            # Go through elements
            myPieces = 1
            holes = 0
            opponent = 0
            pieces = []
            isOpponent = False

            def inBounds(x, y, i):
                return not(((y + i * dy) >= brd.h) or ((y + i * dy) < 0) or ((x + i * dx) >= brd.w) or ((x + i * dx) < 0))

            ##forward
            if inBounds(x, y, 1):
                if brd.board[y + 1*dy][x + 1*dx] != t and brd.board[y + 1*dy][x + 1*dx] != 0: # is opponent
                    #print("Found opponent")
                    pieces.append(brd.board[y + 1*dy][x + 1*dx])
                    opponent += 1
                    i = 2
                    while (i < brd.n) and (inBounds(x, y, i)):
                        if brd.board[y + i*dy][x + i*dx] != t and brd.board[y + i*dy][x + i*dx] != 0:
                            pieces.append(brd.board[y + i*dy][x + i*dx])
                            #print("Found opponent")
                            opponent += 1
                            i +=1
                        else:
                            break
                else:
                    i = 1
                    while (i < brd.n) and (inBounds(x, y, i)):
                        if brd.board[y + i*dy][x + i*dx] != t and brd.board[y + i*dy][x + i*dx] != 0:
                            break
                        elif brd.board[y + i*dy][x + i*dx] == t:
                            pieces.append(brd.board[y + i*dy][x + i*dx])
                            myPieces += 1
                        elif brd.board[y + i*dy][x + i*dx] == 0:
                            pieces.append(brd.board[y + i*dy][x + i*dx])
                            holes += 1
                        i +=1
                    i = 0

            #print("Pieces: ", pieces)
            #return [myPieces, holes, opponent, pieces]
            return pieces

        def calcVal_2(pieces_forward, pieces_backward, t):
            pieces_backward.reverse()
            pieces = pieces_backward + [t] + pieces_forward
            #print("Pieces_2: ", pieces, len(pieces), brd.n)
            max = 0
            sets = []
            for i in range((len(pieces)-(brd.n-1))):
                mine = 0
                hole = 0
                opp = 0
                for j in range(brd.n):
                    if pieces[i+j] == t:
                        mine += 1
                    elif pieces[i+j] == 0:
                        hole += 1
                    else:
                        opp += 1
                sets.append([mine, hole, opp])
            #print("Sets: ", sets)

            count = []
            cnt = 0
            for each in sets:
                if each[2] == brd.n-1:
                    max += 10**10
                elif each[0] == brd.n-1:
                    max += 10**10
                elif each[2] == brd.n-2:
                    max += 10**5
                elif each[0] == brd.n-2:
                    max += 10**5
                else:
                    for i in range(brd.n-3, -1, -1):
                        for j in range(brd.n-3, -1, -1):
                            if each == [i, brd.n-i-j, j]:
                                max += 5**(i*j)
            return max


        y = 0
        for i in range(brd.h-1, -1, -1):
            if brd.board[i][col] != 0:
                y = i
                #print("inserted column and y: ", y)
                break

        h_val = calcVal_2(countDirectionPieces(col, y, 1, 0), countDirectionPieces(col, y, -1, 0), brd.board[y][col])
        v_val = calcVal_2([], countDirectionPieces(col, y, 0, -1), brd.board[y][col])
        du_val = calcVal_2(countDirectionPieces(col, y, 1, 1), countDirectionPieces(col, y, -1, -1), brd.board[y][col])
        dd_val = calcVal_2(countDirectionPieces(col, y, 1, -1), countDirectionPieces(col, y, -1, 1), brd.board[y][col])

        value = h_val + v_val + du_val + dd_val
        #print("Total Value: ", value)
        return value


    def staticEval(self, brd, col, maximizingPlayer):
        board = brd.copy().board
        value = self.heuristic(brd, col, maximizingPlayer)
        return value

    def minimax(self, brd, col, depth, alpha, beta, maximizingPlayer): # First
        if depth == 0 or self.terminalTest(brd):
            #print("Terminal Child")
            return self.staticEval(brd, col, maximizingPlayer), col

        if maximizingPlayer:
            maxEval = -10**15
            best_action = col
            for child in self.get_successors(brd):
                #print("Maximizing Player")
                #print("Child: ", child[1])
                #child[0].print_it()
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                #maxEval = max(maxEval, evalu)
                if max(maxEval, evalu) > maxEval:
                    maxEval = evalu
                    best_action = child[1]

                alpha = max(alpha, evalu)
                if beta <= alpha:
                    #print("Pruned")
                    break
            #print("maxEval: ", maxEval, best_action)
            return maxEval, best_action
        else:
            minEval = 10**15
            best_action = col
            for child in self.get_successors(brd):
                #print("Minimizing Player")
                #print("Child: ", child[1])
                #child[0].print_it()
                evalu, column = self.minimax(child[0], child[1], depth-1, alpha, beta, not(maximizingPlayer))
                #minEval = min(minEval, evalu)
                if min(minEval, evalu) < minEval:
                    minEval = evalu
                    best_action = child[1]

                beta = min(beta, evalu)
                if beta <= alpha:
                    #print("Pruned")
                    break
            #print("minEval: ", minEval, best_action)
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
        #print("Results!: ", value, move)

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

        '''print(freecols, mid)
        print(midfirst)'''

        for col in midfirst:
            # Clone the original board
            nb = brd.copy()
            # Add a token to the new board
            # (This internally changes nb.player, check the method definition!)
            nb.add_token(col)
            # Add board to list of successors
            succ.append((nb, col))

        return succ
