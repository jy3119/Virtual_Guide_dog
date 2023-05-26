import serial
import numpy as np

# Create a serial connection
ser = serial.Serial('COM5', 9600)
ser.flushInput()  # Change to your Arduino's port

# Constants for obstacle analysis
MIN_SAFE_DISTANCE = 50  # Minimum safe distance in cm

# Initialize variables
obstacles = []

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

            # Store obstacle data
            obstacles.append((angle, distance))

            # Calculate obstacle positions in Cartesian coordinates
            x = distance * np.cos(np.radians(angle))
            y = distance * np.sin(np.radians(angle))

            # Perform obstacle analysis
            # Determine safe directions based on obstacle positions and distances
            safe_directions = []
            for obstacle in obstacles:
                obstacle_angle = obstacle[0]
                obstacle_distance = obstacle[1]

                if obstacle_distance >= MIN_SAFE_DISTANCE:
                    safe_directions.append(obstacle_angle)

            # Calculate the safest direction of travel
            safest_direction = None
            if safe_directions:
                safest_direction = np.median(safe_directions)

            # Print the safest direction
            print("Safest Direction:", safest_direction)

ser.close()
