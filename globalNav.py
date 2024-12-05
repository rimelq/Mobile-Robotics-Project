import cv2
import numpy as np
import matplotlib.pyplot as plt
from heapq import heappush, heappop
from functools import partial

SMALL_AREA_THRESHOLD = 1000       # threshold to avoid detection of too small obstacles

# Helper function to display images
def show_image(img, title="Image"):
    plt.figure(figsize=(10, 10))
    if len(img.shape) == 2:  # Grayscale image
        plt.imshow(img, cmap='gray')
    else:
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.title(title)
    plt.axis("on")
    plt.show()
# Preprocess the image for contour detection.
def prepare_image(frame):
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame.copy()
    
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    gray = cv2.medianBlur(gray, 3)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

# Simplify a contour by approximating it to a polygon (corner points).
def approximate_contour(contour, epsilon_factor=0.02):
    epsilon = epsilon_factor * cv2.arcLength(contour, True)
    contour_approx = cv2.approxPolyDP(contour, epsilon, True)
    return contour_approx

# Expands contours outward by a specified distance to account for the robot's size.
def expand_contours(contours, scaling_factor):
    expanded_contours = []
    for cont in contours:
        # Skip small contours
        area = cv2.contourArea(cont)
        if area < SMALL_AREA_THRESHOLD:
            continue
        
        # Simplify contour
        approx = approximate_contour(cont)
        
        # Create a mask for each contour
        x, y, w, h = cv2.boundingRect(approx)
        mask_height = h + scaling_factor * 2 
        mask_width = w + scaling_factor * 2 
        mask = np.zeros((mask_height, mask_width), dtype=np.uint8)
        shift = np.array([[x - scaling_factor - 5, y - scaling_factor - 5]], dtype=np.int32)
        approx_shifted = approx - shift
        cv2.drawContours(mask, [approx_shifted], -1, 255, thickness=cv2.FILLED)
        
        # Dilate the contour to expand it
        kernel_size = scaling_factor * 2 + 1
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        expanded_mask = cv2.dilate(mask, kernel, iterations=1)
        
        # Find the new expanded contour
        contours_expanded, _ = cv2.findContours(expanded_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_expanded:
            # Shift back the contour
            expanded_contour = contours_expanded[0] + shift
            expanded_contours.append(expanded_contour)
    return expanded_contours

# Creates an occupancy grid for pathfinding algorithms based on contours
def create_occupancy_grid(image_shape, contours):
    occupancy_grid = np.zeros(image_shape, dtype=np.uint8)
    # Fill in the obstacles
    cv2.fillPoly(occupancy_grid, contours, 255)
    return occupancy_grid

# Heuristic function for A* algorithm (Euclidean distance)
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

# Performs A* search on the occupancy grid.
def a_star_search(occupancy_grid, start, goal):
    open_set = []
    heappush(open_set, (heuristic(start, goal), 0, start))  # (f_cost, g_cost, position)
    came_from = {}
    g_costs = {start: 0}
    explored = set()

    while open_set:
        current_f_cost, current_g_cost, current_pos = heappop(open_set)
        if current_pos == goal:
            # Reconstruct path
            path = []
            while current_pos in came_from:
                path.append((current_pos[1], current_pos[0]))  # Convert back to (x, y)
                current_pos = came_from[current_pos]
            path.append((start[1], start[0]))  # Start point
            return path[::-1]  # Return reversed path

        explored.add(current_pos)

        # Get neighbors (up, down, left, right)
        neighbors = [
            (current_pos[0] - 1, current_pos[1]),  # Up
            (current_pos[0] + 1, current_pos[1]),  # Down
            (current_pos[0], current_pos[1] - 1),  # Left
            (current_pos[0], current_pos[1] + 1)   # Right
        ]

        for neighbor in neighbors:
            if (0 <= neighbor[0] < occupancy_grid.shape[0]) and (0 <= neighbor[1] < occupancy_grid.shape[1]):
                if occupancy_grid[neighbor[0], neighbor[1]] != 255:
                    tentative_g_cost = g_costs[current_pos] + 1
                    if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                        came_from[neighbor] = current_pos
                        g_costs[neighbor] = tentative_g_cost
                        f_cost = tentative_g_cost + heuristic(neighbor, goal)
                        heappush(open_set, (f_cost, tentative_g_cost, neighbor))
    # If we reach here, no path was found
    return None
    
# Simplifies the path using the Ramer-Douglas-Peucker algorithm to have at most max_points.
def simplify_path_rdp(path, max_points=20):
    # Ramer-Douglas-Peucker algorithm.
    def rdp(points, epsilon):
        if len(points) < 3:
            return points

        # Line from start to end
        start, end = points[0], points[-1]
        line = end - start
        line_norm = np.hypot(line[0], line[1])
        if line_norm == 0:
            dists = np.hypot(*(points - start).T)
        else:
            dists = np.abs(np.cross(line, start - points)) / line_norm

        index = np.argmax(dists)
        max_dist = dists[index]

        if max_dist > epsilon:
            # Recursive call
            left = rdp(points[:index+1], epsilon)
            right = rdp(points[index:], epsilon)
            return np.vstack((left[:-1], right))
        else:
            return np.array([start, end])

    # Convert path to NumPy array
    points = np.array(path)

    # Adjust epsilon to get at most max_points
    epsilon_min = 0
    epsilon_max = np.hypot(*(points[-1] - points[0]))  # Maximum possible distance

    # Binary search to find the right epsilon
    for _ in range(100):
        epsilon = (epsilon_min + epsilon_max) / 2
        simplified = rdp(points, epsilon)
        if len(simplified) > max_points:
            epsilon_min = epsilon
        else:
            epsilon_max = epsilon
        if epsilon_max - epsilon_min < 1e-5:
            break

    # Final simplified path
    simplified = rdp(points, epsilon_max)
    return simplified.astype(int)

# Processes an image to find contours and the shortest path.
def find_shortest_path(image_path, scalingFactor, start, goal, verbose=False):
    # Load the image
    frame = image_path
    if frame is None:
        raise FileNotFoundError(f"Image file not found at {image_path}")

    if verbose:
        show_image(frame, "Original Image")

    # Preprocess the image to get the binary obstacle mask
    obstacle_mask = prepare_image(frame)

    if verbose:
        show_image(obstacle_mask, "Obstacle Mask")

    # Find contours in the obstacle mask
    contours, _ = cv2.findContours(obstacle_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Expand contours to account for the robot's size
    expanded_contours = expand_contours(contours, int(scalingFactor))

    if verbose:
        frame_with_contours = frame.copy()
        
        cv2.drawContours(frame_with_contours, expanded_contours, -1, (0, 0, 255), 2)
        cv2.circle(frame_with_contours, start, radius=10, color=(0, 0, 255), thickness=-1)  # Start in red
        cv2.putText(frame_with_contours, 'Start', (start[0] + 15, start[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.circle(frame_with_contours, goal, radius=10, color=(0, 255, 0), thickness=-1)  # Goal in green
        cv2.putText(frame_with_contours, 'Goal', (goal[0] + 5, goal[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        show_image(frame_with_contours, "Image with Expanded Contours")

    # Create occupancy grid
    occupancy_grid = create_occupancy_grid(obstacle_mask.shape, expanded_contours)

    # Convert start and goal points to integer pixel positions
    start_pos = (int(start[1]), int(start[0]))  # Note the order (row, column)
    goal_pos = (int(goal[1]), int(goal[0]))

    # Perform A* search
    path = a_star_search(occupancy_grid, start_pos, goal_pos)

    if path is None:
        print("No path found.")
        return [], expanded_contours

    # Simplify the path to have at most 20 points
    shortest_path = simplify_path_rdp(path, max_points=20)

    if verbose:
        # Create a color image for visualization
        path_image = cv2.cvtColor((occupancy_grid == 0).astype(np.uint8)*255, cv2.COLOR_GRAY2BGR)

        # Draw the simplified path
        for i in range(len(shortest_path) - 1):
            pt1 = tuple(shortest_path[i])
            pt2 = tuple(shortest_path[i + 1])
            cv2.line(path_image, pt1, pt2, (50, 255, 50), thickness=3)

        # Draw start and goal points
        cv2.circle(path_image, start, radius=10, color=(0, 0, 255), thickness=-1)  # Start in red
        cv2.putText(path_image, 'Start', (start[0] + 15, start[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.circle(path_image, goal, radius=10, color=(0, 255, 0), thickness=-1)  # Goal in green
        cv2.putText(path_image, 'Goal', (goal[0] + 5, goal[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        show_image(path_image, "Simplified Path on Occupancy Grid")

    shortest_path = shortest_path.tolist()    
    return shortest_path, expanded_contours

