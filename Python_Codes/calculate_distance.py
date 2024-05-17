import math

def calculateDistance(pointA: tuple, pointB: tuple):
    x1, y1 = pointA
    x2, y2 = pointB

    squared_diff_x = (x2 - x1) ** 2
    squared_diff_y = (-y2 - -y1) ** 2

    distance = math.sqrt(squared_diff_x + squared_diff_y)
    return distance

def shortestPath(current_loc: tuple, Areas: dict, States: list):
    nearest_area_A = Areas[0] 
    nearest_area_B = Areas[1]
    nearest_area_C = Areas[2]
    nearest_area_D = Areas[3]
    
    area_name = "area_name"
    area_pos = "pos"

    A_xy = nearest_area_A[area_pos] if States[0] else (5000, 5000)
    A_name = nearest_area_A[area_name]
    distanceA = calculateDistance(current_loc, A_xy)

    B_xy = nearest_area_B[area_pos] if States[1] else (5000, 5000)
    B_name = nearest_area_B[area_name]
    distanceB = calculateDistance(current_loc, B_xy)

    C_xy = nearest_area_C[area_pos] if States[2] else (5000, 5000)
    C_name = nearest_area_C[area_name]
    distanceC = calculateDistance(current_loc, C_xy)

    D_xy = nearest_area_D[area_pos] if States[3] else (5000, 5000)
    D_name = nearest_area_D[area_name]
    distanceD = calculateDistance(current_loc, D_xy)
    
    shortest_path = min(distanceA, distanceB, distanceC, distanceD)
    shortest_path_name = ""

    if shortest_path == distanceA:
        shortest_path_name = A_name
    elif shortest_path == distanceB:
        shortest_path_name = B_name
    elif shortest_path == distanceC:
        shortest_path_name = C_name
    else:
        shortest_path_name = D_name

    print(f"\n[ Shortest Distance: {shortest_path_name} ]\n")
    return shortest_path_name

def thelessCrowdedArea(numClsFnd: tuple, areas: tuple) -> list:
    minimum = min(numClsFnd)
    lessCrowdedArea = [name for value, name in zip(numClsFnd, areas) if value == minimum]
    convertToListOfBool = [False] * len(areas)

    for index in range(len(areas)):
        if areas[index] in lessCrowdedArea:
            convertToListOfBool[index] = True

    print(convertToListOfBool)
    return convertToListOfBool
# print(theShortestDistance(((233, 652)), TableA_closeArea[0], TableA_closeArea[1], TableA_closeArea[2], TableA_closeArea[3]))