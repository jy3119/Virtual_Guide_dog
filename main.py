import serial
import numpy as np
from KalmanFilter import KalmanFilter
import mapping
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flushInput()

#Kalman filter
kf1 = KalmanFilter()
kf2 = KalmanFilter()
kf3 = KalmanFilter()
kf4 = KalmanFilter()

# Constants
resolution = 1  # meters per grid cell
max_distance = 10  # meters
initial_prob = 0.5

# Initialize the occupancy grid
grid_size = int(2 * max_distance / resolution)
occupancy_grid = np.zeros((grid_size, grid_size))

def is_float(value):
    try:
        float(value)
        return True
    except ValueError:
        return False

try:
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        # print("Raw data:", line)

        if not line:
            continue

        if line:
            distances = line.split(',')
            if len(distances) == 4 and all(is_float(d) for d in distances):
                try:
                    distance1 = float(distances[0])
                    distance2 = float(distances[1])
                    distance3 = float(distances[2])
                    distance4 = float(distances[3])

                    fd1 = kf1.filter(distance1)
                    fd2 = kf2.filter(distance2)
                    fd3 = kf3.filter(distance3)
                    fd4 = kf4.filter(distance4)

            
            # Robot position and sensor angles
                    robot_position = (0, 0)
                    sensor_angles = [0, np.pi / 4, np.pi / 2, 3 * np.pi / 4]
                    
                    cartesian_coordinates = mapping.distance_to_cartesian_coordinates(robot_position, [fd1, fd2, fd3, fd4], sensor_angles)

                    # Update the occupancy grid with new measurements
                    mapping.update_occupancy_grid(occupancy_grid, robot_position, cartesian_coordinates, grid_size, resolution)

                    # Visualize the occupancy grid
                    mapping.visualize_occupancy_grid(occupancy_grid)

                    # time.sleep(0.1)
                except ValueError:
                    print("Invalid data received. Skipping...")


        # distances = line.split(',')
        # print(distances)


except KeyboardInterrupt:
    ser.close()                                          