import serial
import time

#Kalman filter parameters
Q = 0.01    #Process noise covariance
R = 10      #Measurement noise covariance
x = 0       #Initial state estimate
P = 1       #Initial covariance estimate
K = 0       #Kalman gain

ser = serial.Serial('COM5', 9600)
ser.flushInput()

try:
    while True:
        ser_bytes = ser.readline()
        distance_str = ser_bytes.strip().decode('ascii')
        distance = float(distance_str.split(',')[0])

        x = x
        P = P + Q

        K = P / (P + R)
        x = x + K * (distance - x)
        P = (1 - K) * P
        
        print("Distance:", distance, "cm")
        time.sleep(0.1)
except KeyboardInterrupt:
    ser.close()                                          