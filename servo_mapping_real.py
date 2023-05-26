import serial
import numpy as np
import matplotlib.pyplot as plt

# Set up the serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)

# Define grid parameters
grid_size = (200, 200)  # Size of the occupancy grid
cell_size = 1.0  # Size of each grid cell in meters

# Initialize the occupancy grid
occupancy_grid = np.zeros(grid_size)

# Create a figure for visualization
fig, ax = plt.subplots()

# Main loop
while True:
    # Read data from Arduino
    line = ser.readline().decode().strip()  # Read a line from serial
    data = line.split(",")  # Split the line into angle and distance values
    angle = float(data[0])
    distance = float(data[1])

    # Convert polar coordinates to Cartesian coordinates
    x = distance * np.cos(np.radians(angle))
    y = distance * np.sin(np.radians(angle))

    # Calculate the grid cell coordinates
    cell_x = int(x / cell_size)
    cell_y = int(y / cell_size)

    # Update the occupancy grid
    if 0 <= cell_x < grid_size[0] and 0 <= cell_y < grid_size[1]:
        occupancy_grid[cell_y, cell_x] = 1

    # Visualize the occupancy grid
    ax.imshow(occupancy_grid, cmap='gray', origin='lower')

    # Refresh the plot
    plt.pause(0.01)

# Close the serial connection
ser.close()
