import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# Initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size=(640, 480))

# Load pre-trained cascade classifiers for wall and staircase detection
wall_cascade = cv2.CascadeClassifier("path/to/wall_cascade.xml")
staircase_cascade = cv2.CascadeClassifier("path/to/staircase_cascade.xml")

# Allow the camera to warm up
time.sleep(0.1)

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # Retrieve the current frame as an array
    image = frame.array

    # Convert the frame to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect walls
    walls = wall_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    for (x, y, w, h) in walls:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Detect staircases
    staircases = staircase_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
    for (x, y, w, h) in staircases:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

    # Display the resulting image with detected obstacles
    cv2.imshow("Obstacle Detection", image)

    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
camera.close()
cv2.destroyAllWindows()
