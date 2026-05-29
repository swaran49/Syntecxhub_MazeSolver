from pyamaze import maze, agent, textLabel
from queue import PriorityQueue
import time

# =========================
# HEURISTIC FUNCTION
# =========================

def h(cell1, cell2):

    x1, y1 = cell1
    x2, y2 = cell2

    return abs(x1 - x2) + abs(y1 - y2)

# =========================
# A* SEARCH ALGORITHM
# =========================

def AStar(m):

    start = (m.rows, m.cols)

    visited_nodes = 0

    g_score = {cell: float('inf') for cell in m.grid}
    g_score[start] = 0

    f_score = {cell: float('inf') for cell in m.grid}
    f_score[start] = h(start, (1, 1))

    open = PriorityQueue()

    open.put((
        h(start, (1, 1)),
        h(start, (1, 1)),
        start
    ))

    aPath = {}

    while not open.empty():

        currCell = open.get()[2]

        visited_nodes += 1

        # Goal reached
        if currCell == (1, 1):
            break

        for d in 'ESNW':

            if m.maze_map[currCell][d] == True:

                # EAST
                if d == 'E':
                    childCell = (
                        currCell[0],
                        currCell[1] + 1
                    )

                # WEST
                if d == 'W':
                    childCell = (
                        currCell[0],
                        currCell[1] - 1
                    )

                # NORTH
                if d == 'N':
                    childCell = (
                        currCell[0] - 1,
                        currCell[1]
                    )

                # SOUTH
                if d == 'S':
                    childCell = (
                        currCell[0] + 1,
                        currCell[1]
                    )

                temp_g_score = g_score[currCell] + 1

                temp_f_score = (
                    temp_g_score +
                    h(childCell, (1, 1))
                )

                if temp_f_score < f_score[childCell]:

                    g_score[childCell] = temp_g_score

                    f_score[childCell] = temp_f_score

                    open.put((
                        temp_f_score,
                        h(childCell, (1, 1)),
                        childCell
                    ))

                    aPath[childCell] = currCell

    # =========================
    # FORWARD PATH
    # =========================

    fwdPath = {}

    cell = (1, 1)

    while cell != start:

        fwdPath[aPath[cell]] = cell

        cell = aPath[cell]

    return fwdPath, visited_nodes

# =========================
# CREATE MAZE
# =========================

m = maze(7, 7)

# Generate random maze
m.CreateMaze(loopPercent=30)

# =========================
# TIMER START
# =========================

start_time = time.time()

# Run A*
path, visited_nodes = AStar(m)

# =========================
# TIMER END
# =========================

end_time = time.time()

execution_time = round(
    end_time - start_time,
    5
)

# =========================
# MOVING AGENT
# =========================

# Blue moving agent
a = agent(
    m,
    footprints=True,
    filled=True,
    color='blue'
)

# Goal marker
goal = agent(
    m,
    1,
    1,
    footprints=True,
    filled=True,
    color='red'
)

# =========================
# TRACE PATH
# =========================

m.tracePath(
    {a: path},
    delay=150
)

# =========================
# LABELS
# =========================

textLabel(
    m,
    'Algorithm',
    'A* Search'
)

textLabel(
    m,
    'Path Length',
    len(path) + 1
)

textLabel(
    m,
    'Visited Nodes',
    visited_nodes
)

textLabel(
    m,
    'Execution Time',
    str(execution_time) + " sec"
)

# =========================
# RUN MAZE
# =========================

m.run()