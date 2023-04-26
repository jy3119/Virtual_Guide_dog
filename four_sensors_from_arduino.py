import serial
from KalmanFilter import KalmanFilter
import time

ser = serial.Serial('/dev/ttyACM0', 9600)
ser.flushInput()

#Kalman filter
kf1 = KalmanFilter()
kf2 = KalmanFilter()
kf3 = KalmanFilter()
kf4 = KalmanFilter()

try:
    while True:
        line = ser.readline().decode('utf-8').strip()

        # distances = line.split(',')
        # print(distances)

        distance1 = kf1.filter(float(line.split(',')[0]))
        distance2 = kf2.filter(float(line.split(',')[1]))
        distance3 = kf3.filter(float(line.split(',')[2]))
        distance4 = kf4.filter(float(line.split(',')[3]))

 
        print("Filtered Distance 1: {distance1:.2f} cm")
        print("Filtered Distance 2: {distance2:.2f} cm")
        print("Filtered Distance 3: {distance3:.2f} cm")
        print("Filtered Distance 4: {distance4:.2f} cm")

        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()                                          