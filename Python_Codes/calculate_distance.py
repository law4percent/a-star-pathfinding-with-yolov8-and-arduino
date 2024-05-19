import math
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement

def calculateDistance(pointA: tuple, pointB: tuple):
    x1, y1 = pointA
    x2, y2 = pointB

    squared_diff_x = (x2 - x1) ** 2
    squared_diff_y = (-y2 - -y1) ** 2

    distance = math.sqrt(squared_diff_x + squared_diff_y)
    return distance

def lessCrowdedArea(numClsFnd: tuple, areas: tuple) -> list:
    minimum = min(numClsFnd)
    lessCrowdedArea = [name for value, name in zip(numClsFnd, areas) if value == minimum]
    convertToListOfBool = [False] * len(areas)

    for index in range(len(areas)):
        if areas[index] in lessCrowdedArea:
            convertToListOfBool[index] = True

    print(convertToListOfBool)
    return convertToListOfBool

def CreatePath(start_XY: tuple, end_XY: tuple, _matrix, display_demo = True):
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
        print("======= Demo Map =======\n")
        print(grid.grid_str(path=path, start=start_area, end=end_area))
        print("\n======= Demo Map End =======\n")
    return path
# print(theShortestDistance(((233, 652)), TableA_closeArea[0], TableA_closeArea[1], TableA_closeArea[2], TableA_closeArea[3]))