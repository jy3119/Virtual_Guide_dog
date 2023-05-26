import serial
import numpy as np
import matplotlib.pyplot as plt

# Create a serial connection
ser = serial.Serial('COM5', 9600)  # Change to the correct port

# Grid parameters
GRID_SIZE = 20

# Create an empty occupancy grid
occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))

# Read the occupancy grid data from Arduino
while True:
    # Read a line from the serial port
    line = ser.readline().decode('utf-8').strip()

    # Check if the line contains occupancy grid data
    if line.startswith('Occupancy Grid:'):
        # Clear the previous occupancy grid
        occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))

        # Read the occupancy grid data
        for i in range(GRID_SIZE):
            line = ser.readline().decode('utf-8').strip()
            row_data = line.split(' ')
            for j in range(GRID_SIZE):
                occupancy_grid[i][j] = int(row_data[j])

        # Visualize the occupancy grid
        plt.imshow(occupancy_grid, cmap='binary', origin='lower')
        plt.title('Occupancy Grid')
        plt.xlabel('Column')
        plt.ylabel('Row')
        plt.xticks(range(GRID_SIZE))
        plt.yticks(range(GRID_SIZE))
        plt.grid(color='gray', linestyle='-', linewidth=0.5)
        plt.colorbar(label='Occupancy')
        plt.show()

ser.close()