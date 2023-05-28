import serial
import numpy as np
import matplotlib.pyplot as plt

# Configure the serial port
serial_port = '/dev/ttyACM0'  # Replace with the appropriate port for your Arduino
baud_rate = 9600

# Define grid parameters
grid_size = 100  # Number of cells in each dimension
cell_size = 1.0  # Size of each cell in meters
distance_threshold = 0.5  # Threshold distance for considering a cell as occupied

# Initialize occupancy grid map
occupancy_grid = np.zeros((grid_size, grid_size))  # Initialize with zeros (unknown occupancy)

# Smoothening/filtering parameters
window_size = 3  # Size of the moving average window

# Configure the serial port
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Read data from Arduino
def read_data_from_arduino():
    data = ser.readline().decode().strip()
    angle, distance = parse_data(data)
    return angle, distance

# Parse the data received from Arduino
def parse_data(data):
    values = data.split(',')
    angle = []
    distance = []

    for i in range(0, len(values), 2):
        if i+1 >= len(values):
            break

        angle_str = values[i].strip()
        distance_str = values[i+1].strip()

        if angle_str == '' or distance_str == '':
            continue

        try:
            angle.append(int(angle_str))
            distance.append(float(distance_str))
        except ValueError:
            continue

    angle = np.array(angle)
    distance = np.array(distance)

    # Remove outliers where distances > 100cm
    valid_indices = np.where(distance <= 100)[0]
    angle = angle[valid_indices]
    distance = distance[valid_indices]

    return angle, distance

# Apply a simple moving average filter to the distance array
def apply_moving_average_filter(distance_array, window_size):
    smoothed_distances = []
    half_window = window_size // 2

    for i in range(len(distance_array)):
        start_index = max(0, i - half_window)
        end_index = min(len(distance_array), i + half_window + 1)
        neighbors = distance_array[start_index:end_index]
        smoothed_distances.append(sum(neighbors) / len(neighbors))

    return smoothed_distances

# Update the occupancy grid with obstacle information
def update_occupancy_grid(angle, distance):
    # Apply moving average filter to distance values
    smoothed_distances = apply_moving_average_filter(distance, window_size)

    for i in range(len(smoothed_distances)):
        smoothed_distance = smoothed_distances[i]
        x = int(smoothed_distance * np.cos(np.deg2rad(angle[i])) / cell_size + grid_size // 2)
        y = int(smoothed_distance * np.sin(np.deg2rad(angle[i])) / cell_size + grid_size // 2)
        if x >= 0 and x < grid_size and y >= 0 and y < grid_size:
            occupancy_grid[x, y] = 1

# Estimate obstacle size and location based on occupancy grid map
def estimate_obstacles():
    obstacles = []
    obstacle = None
    for i in range(grid_size):
        for j in range(grid_size):
            if occupancy_grid[i, j] == 1:  # Occupied cell
                if obstacle is None:
                    obstacle = {'start_x': i, 'start_y': j, 'end_x': i, 'end_y': j}
                else:
                    obstacle['end_x'] = i
                    obstacle['end_y'] = j
            else:  # Unoccupied cell
                if obstacle is not None:
                    obstacles.append(obstacle)
                    obstacle = None
    if obstacle is not None:
        obstacles.append(obstacle)
    return obstacles

# Visualize the occupancy grid map with estimated obstacles
def visualize_map_with_obstacles(obstacles):
    plt.figure(figsize=(8, 8))
    plt.imshow(occupancy_grid, cmap='binary', origin='lower', extent=(-50, 50, -50, 50))
    for obstacle in obstacles:
        start_x = (obstacle['start_x'] - grid_size // 2) * cell_size
        start_y = (obstacle['start_y'] - grid_size // 2) * cell_size
        end_x = (obstacle['end_x'] - grid_size // 2) * cell_size
        end_y = (obstacle['end_y'] - grid_size // 2) * cell_size
        plt.plot([start_y, end_y], [start_x, end_x], 'r-', linewidth=2)
    plt.xlabel('Y (m)')
    plt.ylabel('X (m)')
    plt.title('Occupancy Grid Map with Estimated Obstacles')
    plt.grid(True)
    plt.show()

# Main loop
while True:
    # Read data from Arduino
    angle, distance = read_data_from_arduino()

    # Update the occupancy grid map
    update_occupancy_grid(angle, distance)

    # Estimate obstacles based on the occupancy grid map
    obstacles = estimate_obstacles()

    # Visualize the map with estimated obstacles
    visualize_map_with_obstacles(obstacles)
