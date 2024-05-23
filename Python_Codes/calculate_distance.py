import math
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

def calculateDistance(pointA: tuple, pointB: tuple):
    x1, y1 = pointA
    x2, y2 = pointB

    squared_diff_x = (x2 - x1) ** 2
    squared_diff_y = (y2 - y1) ** 2

    distance = math.sqrt(squared_diff_x + squared_diff_y)
    return distance

def CreatePath(start_XY: tuple, end_XY: tuple, _matrix, display_demo=True):
    grid = Grid(matrix=_matrix)
    grid.cleanup()
    start_x = start_XY[0]
    start_y = start_XY[1]
    end_x = end_XY[0]
    end_y = end_XY[1]

    start_area = grid.node(start_x, start_y) # x y
    end_area = grid.node(end_x, end_y) # x y


    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start_area, end_area, grid)

    if display_demo:
        print("\n======= Demo Map =======\n")
        print(grid.grid_str(path=path, start=start_area, end=end_area))
        print("\n======= Demo Map End =======\n")
    return path

def convertPathToDirection(path, row_max, col_max, display=True) -> list:
    direction_list = []
    x_max = col_max
    y_max = row_max
    path_len = len(path)
    # if path_len 

    for index in range(path_len - 1):
        next_path = path[index + 1]
        current_x = path[index][0]
        current_y = path[index][1]
        done = False

        if current_x - 1 >= 0 and not done:
            if next_path == (current_x - 1, current_y):
                direction_list.append("L")
                done = True

        if current_x + 1 < x_max and not done:
            if next_path == (current_x + 1, current_y):
                direction_list.append("R")
                done = True

        if current_y - 1 >= 0 and not done:
            if next_path == (current_x, current_y - 1):
                direction_list.append("T")
                done = True

        if current_y + 1 < y_max and not done:
            if next_path == (current_x, current_y + 1) :
                direction_list.append("B")
                done = True

    if path_len > 1:
        next_path = path[path_len - 1]
        current_x = path[path_len - 1][0]
        current_y = path[path_len - 1][1]
        done = False

        if current_x - 1 >= 0 and not done:
            if next_path == (current_x - 1, current_y):
                direction_list.append("L")
                done = True

        if current_x + 1 < x_max and not done:
            if next_path == (current_x + 1, current_y):
                direction_list.append("R")
                done = True

        if current_y - 1 >= 0 and not done:
            if next_path == (current_x, current_y - 1):
                direction_list.append("T")
                done = True

        if current_y + 1 < y_max and not done:
            if next_path == (current_x, current_y + 1) :
                direction_list.append("B")
                done = True

    if display:
        print(direction_list)
    return direction_list
# print(theShortestDistance(((233, 652)), TableA_closeArea[0], TableA_closeArea[1], TableA_closeArea[2], TableA_closeArea[3]))