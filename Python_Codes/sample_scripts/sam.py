import math

# Define the coordinates of the areas
areas = {
    'A': (233, -175),
    'B': (636, -175),
    'C': (1050, -175),
    'D': (233, -400),
    'E': (636, -400),
    'F': (1050, -400),
    'G': (233, -652),
    'H': (636, -652),
    'I': (1050, -652)
}

# Define the number of people in each area
crowd_density = {
    'A': 10,
    'B': 20,
    'C': 5,
    'D': 15,
    'E': 8,
    'F': 25,
    'G': 12,
    'H': 7,
    'I': 18
}

# Function to calculate the Euclidean distance
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to find the path with the least crowd density and shortest distance
def find_optimal_path(current_location, target_location, intermediate_areas):
    current_x, current_y = current_location
    target_x, target_y = target_location
    optimal_path = []
    min_score = float('inf')
    optimal_area = None

    for area in intermediate_areas:
        area_x, area_y = areas[area]
        distance_to_area = euclidean_distance(current_x, current_y, area_x, area_y)
        distance_to_target = euclidean_distance(area_x, area_y, target_x, target_y)
        total_distance = distance_to_area + distance_to_target
        people_count = crowd_density[area]

        # Combine distance and crowd density into a single score
        score = total_distance + people_count  # You can adjust this formula as needed

        if score < min_score:
            min_score = score
            optimal_area = area
            optimal_path = [(current_x, current_y), (area_x, area_y), (target_x, target_y)]

    return optimal_path, optimal_area, min_score

# Example usage
current_location = (223, -175)
target_location = (636, -400)
intermediate_areas = ['B', 'D']
optimal_path, optimal_area, score = find_optimal_path(current_location, target_location, intermediate_areas)

# Display the path with area tags
print(f"The optimal path is from your current location to area {optimal_area} and then to the target location.")
print(f"Path coordinates: {optimal_path}")
print(f"Score: {score:.2f}")
