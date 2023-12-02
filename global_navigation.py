# Global Navigation Function, basics of mobile robotics project 21.11.23 Corentin Jossi
# Code created by using the basics of mobile robotics courses, stackoverflow, youtube and chatGPT
# Inputs 
# obstacles :       2xNxA   The coordinates of N number of Obstacles and A angles
# robot_position :  2x1     The coordinates of the center of the robot
# reset :           bool    Recalculate the path or not ?
# end_point :       2x1     The coordinates of the end point
#
# Outputs:
# path :            Mx2     The coordinates of M points between the position of the robot and the end point

# Import
import math
import heapq

# Function to transform the obstacles into a list of points
def obstacles_to_points(obstacles, robot_position, end_point):
    #initialise points
    points = []

    # Add the robot position as the first point
    points.append(robot_position)

    # Loop through all obstacles
    for obst in obstacles:
        # Loop through all points of the obstacle
        for i in range(len(obst)):
            points.append(obst[i])


    # Add the end point in the list
    points.append(end_point)
    
    return points

# Fonction that check if 
def do_intersects_obstacles(point_a, point_b, obstacles):
    # Loop through all obstacles
    for obst in obstacles:
        # Loop through all points in the obstacles
        for i in range(len(obst)):
            obst_point_a = obst[i]
            obst_point_b = obst[(i + 1) % len(obst)]

            # if there is an intersection, the 2 points are not visibles -> no need to continue
            if intersect(point_a, point_b, obst_point_a, obst_point_b):
                return True
            
            # Special Case I : If the points are from the obstacles
            # Check if the points are points of the obstacle
            if point_a in obst and point_b in obst:
                # Check if the points are an edge of the obstacle or not
                point_a_index_plus_one = (obst.index(point_a) + 1) % len(obst)
                point_a_index_minus_one = (obst.index(point_a) + len(obst) - 1) % len(obst)
    
                if not (point_b == obst[(point_a_index_plus_one)] or 
                        point_b == obst[(point_a_index_minus_one)]):
                    
                    return True

        # Special Case II : If 2 points of the obstacles are in the same line as the point
        for i in range(len(obst)):
            if on_segment(point_a, point_b, obst[i]) and (point_a != obst[i] and point_b != obst[i]):
                return True
            
    # If there is absolutely no intersection, we can return false
    return False

# ----Here we use a simple way to check if the 2 segments intersects or not----
# This methode was taken from : https://stackoverflow.com/questions/3838329/how-can-i-check-if-two-segments-intersect
# I implemented this option because I find it nice an simpler than my older implementation
# It has a problem however, if 2 segments are in the same line, the result is incorrect
def intersect(point_a, point_b, obst_point_a, obst_point_b):
    # Take the x and y
    a_x, a_y = point_a
    b_x, b_y = point_b
    obst_a_x, obst_a_y = obst_point_a
    obst_b_x, obst_b_y = obst_point_b
    
    dx = b_x - a_x
    dx_obst = obst_b_x - obst_a_x
    dy = b_y - a_y
    dy_obst = obst_b_y - obst_a_y
    
    p0 = dy_obst * (obst_b_x - a_x) - dx_obst * (obst_b_y - a_y)
    p1 = dy_obst * (obst_b_x - b_x) - dx_obst * (obst_b_y - b_y)
    p2 = dy * (b_x - obst_a_x) - dx * (b_y - obst_a_y)
    p3 = dy * (b_x - obst_b_x) - dx * (b_y - obst_b_y)
    
    return (p0 * p1 < 0) & (p2 * p3 < 0)

# ---End of the elegent way to check if the 2 segments intersects or not---

# Function that check if the point C is between the point A and the point B
# Use the implementation from 
# https://stackoverflow.com/questions/328107/how-can-you-determine-a-point-is-between-two-other-points-on-a-line-segment
def on_segment(a, b, c):
    a_x, a_y = a
    b_x, b_y = b
    c_x, c_y = c
    epsilon = 1e-5
    cross_product = (c_y - a_y) * (b_x - a_x) - (c_x - a_x) * (b_y - a_y)

    # For the floating point
    if abs(cross_product) > epsilon:
        return False

    dot_product = (c_x - a_x) * (b_x - a_x) + (c_y - a_y) * (b_y - a_y)
    if dot_product < 0:
        return False

    squared_length_ba = (b_x - a_x) * (b_x - a_x) + (b_y - a_y) * (b_y - a_y)
    if dot_product > squared_length_ba:
        return False

    return True

# Function that compute the euclidian distance between 2 points
def point_to_point_distance(point_a, point_b):
    # Simply compute the Euclidean distance
    x_coordinates = point_a[0] - point_b[0]
    y_coordinates = point_a[1] - point_b[1]
    return math.sqrt(x_coordinates ** 2 + y_coordinates ** 2)
    
def greater_obstacles(obstacles, margin):
    greater_obstacles = []
    
    for obst in obstacles:
        # Centroid
        center_x = sum(point[0] for point in obst) / len(obst)
        center_y = sum(point[1] for point in obst) / len(obst)

        # Move the obstacle to the origin
        centered_obstacle = tuple((point[0] - center_x, point[1] - center_y) for point in obst)
    
        # Resize
        scaled_obstacle = tuple((point[0] * 1.5, point[1] * 1.5) for point in centered_obstacle)
    
        # Replace the obstacle where it belongs
        final_obstacle = tuple((point[0] + center_x, point[1] + center_y) for point in scaled_obstacle)

        greater_obstacles.append(final_obstacle)
        
    return greater_obstacles
    
def visibility_graph(obstacles, robot_position, end_point, margin):
    # Creation of the visibility graph
    visibility_graph = {}
    
    # Make the obstacles slightly bigger, so the robot will not collide on them
    obstacles = greater_obstacles(obstacles, margin)
    
    # Get point from obstacles
    points = obstacles_to_points(obstacles, robot_position, end_point)

    # Main loop, Check if a point can see all the other point
    for point_a in points:
        for point_b in points:
            if point_a != point_b and not do_intersects_obstacles(point_a, point_b, obstacles):
                # Add the point a to the visibility graph if it does not exist
                if point_a not in visibility_graph:
                    visibility_graph[point_a] = []

                # Compute the distance between the 2 points, for the future path finding
                distance = point_to_point_distance(point_a, point_b)   
                
                # Add the other point to the point a and give the distance
                visibility_graph[point_a].append((point_b, distance))
    
    return visibility_graph


# standard implementation of A3 algorythm

def heuristic_cost_estimate(current, goal):
    # A simple heuristic: Euclidean distance between current and goal
    return math.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)

def a_star(graph, start, goal):
    queue = [(0 + heuristic_cost_estimate(start, goal), 0, start, [])]

    visited = set()

    while queue:
        (_, cost, node, path) = heapq.heappop(queue)

        if node not in visited:
            visited.add(node)
            path = path + [node]

            if node == goal:
                return path
            for neighbor, distance in graph[node]:
                if neighbor not in visited:
                    new_cost = cost + distance
                    heapq.heappush(queue, (new_cost + heuristic_cost_estimate(neighbor, goal), new_cost, neighbor, path))

    return None

# Main function
def global_navigation(obstacles,robot_position,end_point):
    # Margin to make the obstacles larger
    margin = 1.1

    graph = visibility_graph(obstacles, robot_position, end_point, margin)
    return a_star(graph, robot_position, end_point)
