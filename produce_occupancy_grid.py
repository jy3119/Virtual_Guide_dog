import serial
import numpy as np
import matplotlib.pyplot as plt

# Create a serial connection
ser = serial.Serial('COM5', 9600)  # Change port as per your setup

# Create a grid map
grid_size = 200  # size of the grid (cm)
grid = np.zeros((grid_size, grid_size))

# Create a figure for visualization
plt.figure()
ax = plt.gca()

while True:
    # Reset the grid for each rotation
    grid.fill(0)

    for pos in range(181):
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()

        # Split the line into angle and distance
        parts = line.split(',')
        if len(parts) == 2:
            angle_str, distance_str = parts

            angle = float(angle_str)
            distance = float(distance_str)

            # Convert angle from degrees to radians
            angle_rad = np.radians(angle)

            # Convert polar coordinates to Cartesian coordinates
            x = distance * np.cos(angle_rad)
            y = distance * np.sin(angle_rad)

            # Convert real-world coordinates to grid coordinates
            grid_x = int(grid_size / 2 + x)
            grid_y = int(grid_size / 2 + y)

            # Update the grid cell if it's within the grid bounds
            if 0 <= grid_x < grid_size and 0 <= grid_y < grid_size:
                grid[grid_x][grid_y] = 1

    # Visualize the occupancy grid
    ax.imshow(grid, cmap='binary', origin='lower')

    plt.draw()
    plt.pause(0.001)
    plt.clf()

ser.close()
