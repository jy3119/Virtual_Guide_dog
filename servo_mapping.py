import serial
import numpy as np
import matplotlib.pyplot as plt

# Create a serial connection
ser = serial.Serial('COM5', 9600)
ser.flushInput()  # Change to your Arduino's port

# Create a grid map
grid_size = 200  # Size of the grid (cm)
grid = np.zeros((grid_size, grid_size))

# Create a figure for visualization
fig, ax = plt.subplots()

# Run the loop indefinitely
while True:
    # Read a line from the serial port
    line = ser.readline().decode('utf-8').strip()

    # Split the line into angle and distance
    parts = line.split(',')
    if len(parts) == 2:
        angle_str, distance_str = parts

        # Extract the numeric values
        angle_split = angle_str.split(':')
        distance_split = distance_str.split(':')
        if len(angle_split) > 1 and len(distance_split) > 1:
            angle = float(angle_split[1].strip())
            distance = float(distance_split[1].strip())

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
    ax.imshow(grid, cmap='gray', origin='lower')

    # Refresh the plot
    plt.pause(0.01)

ser.close()

