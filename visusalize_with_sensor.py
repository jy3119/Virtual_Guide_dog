import serial
import numpy as np
import matplotlib.pyplot as plt

# Create a serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)  # Change to the correct port

# Grid parameters
GRID_SIZE = 10

# Create an empty occupancy grid
occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))

# Initialize the sensor position
sensor_position = (0, 0)

# Read the occupancy grid data from Arduino
while True:
    # Read a line from the serial port
    line = ser.readline().decode('utf-8').strip()

    # Check if the line contains occupancy grid data
    if line.startswith('Occupancy Grid:'):
        # Clear the previous occupancy grid
        occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))

        # Read the occupancy grid values
        for i in range(GRID_SIZE):
            row_data = ser.readline().decode('utf-8').strip().split(' ')
            for j in range(GRID_SIZE):
                occupancy_grid[i][j] = int(row_data[j])

        # Plot the occupancy grid
        plt.imshow(occupancy_grid, cmap='binary', origin='lower')

        # Plot the sensor position marker
        plt.plot(sensor_position[1], sensor_position[0], 'ro', markersize=10)

        # Show the plot
        plt.show()

ser.close()