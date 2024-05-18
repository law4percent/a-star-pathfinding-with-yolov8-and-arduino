from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

def ConvertToMatrix_Array_of_Array(_array):
    rows = 6
    columns = 8
    matrix = [[0] * columns for _ in range(rows)]
    
    _row = 0
    _column = 0

    for index in range(len(_array)):
        matrix[_row][_column] = _array[index]
        _column += 1
        if _column == columns:
            _column = 0
            _row += 1

    return matrix


binary_map = [
            #   0  1  2  3  4  5  6  7
                1, 1, 1, 1, 1, 1, 1, 1, # 0
                1, 0, 0, 1, 1, 0, 0, 1, # 1
                1, 0, 0, 1, 1, 0, 0, 1, # 2
                1, 1, 1, 1, 1, 1, 1, 1, # 3
                1, 0, 0, 1, 1, 0, 0, 1, # 4
                1, 1, 1, 1, 1, 1, 1, 1, # 5
            ]

_matrix = ConvertToMatrix_Array_of_Array(binary_map)

grid = Grid(matrix=_matrix)
grid.cleanup()
start_x = 2
start_y = 0
end_x = 5
end_y = 7

start_area = grid.node(start_x, start_y) # x y
end_area = grid.node(end_x, end_x) # x y

finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
path, runs = finder.find_path(start_area, end_area, grid)
print(runs)
print(grid.grid_str(path=path, start=start_area, end=end_area))
print(len(path))
# path_list= []
# for i in range(len(path)):
#     path_list.append(tuple(path[i]))

# print(path_list)
# print(len(path))