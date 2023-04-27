import numpy as np
import matplotlib.pyplot as plt


# Function to convert distances to Cartesian coordinates
def distance_to_cartesian_coordinates(robot_position, distances, angles):
    coordinates = []
    for distance, angle in zip(distances, angles):
        x = robot_position[0] + distance * np.cos(angle)
        y = robot_position[1] + distance * np.sin(angle)
        coordinates.append((x, y))
    return coordinates

# Function to update the occupancy grid with new measurements
def update_occupancy_grid(occupancy_grid, robot_position, cartesian_coordinates, grid_size, resolution):
    for coord in cartesian_coordinates:
        x, y = coord
        grid_x = int(x / resolution + grid_size // 2)
        grid_y = int(y / resolution + grid_size // 2)

        if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
            occupancy_grid[grid_y, grid_x] += 1

# Function to visualize the occupancy grid
def visualize_occupancy_grid(occupancy_grid):
    plt.imshow(occupancy_grid, cmap='gray_r')
    plt.colorbar()
    plt.show()

def init_occupancy_grid(grid_size):
    return np.zeros((grid_size, grid_size))


# # Main loop
# while True:
#     line = ser.readline().decode(errors='ignore').strip()

#     if not line:
#         continue

#     distances = line.split(',')
#     if len(distances) == 4:
#         try:
#             distance1 = float(distances[0])
#             distance2 = float(distances[1])
#             distance3 = float(distances[2])
#             distance4 = float(distances[3])

#             # Robot position and sensor angles
#             robot_position = (0, 0)
#             sensor_angles = [0, np.pi / 2, np.pi, 3 * np.pi / 2]

#             # Convert distances to Cartesian coordinates
#             cartesian_coordinates = distance_to_cartesian_coordinates(robot_position, [distance1, distance2, distance3, distance4], sensor_angles)

#             # Update the occupancy grid with new measurements
#             update_occupancy_grid(occupancy_grid, robot_position, cartesian_coordinates, grid_size, resolution)

#             # Visualize the occupancy grid
#             visualize_occupancy_grid(occupancy_grid)

#         except ValueError:
#             print("Invalid data received. Skipping...")