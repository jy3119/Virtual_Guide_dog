import cv2
import numpy as np
from picamera.array import PiRGBArray
from picamera import PiCamera

# Initialize the Raspberry Pi camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 30
raw_capture = PiRGBArray(camera, size=(640, 480))

# Allow the camera to warm up
time.sleep(0.1)

for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # Retrieve the current frame as an array
    image = frame.array

    # Convert the frame to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Apply edge detection using the Canny algorithm
    edges = cv2.Canny(gray, threshold1=50, threshold2=150)

    # Perform contour detection on the edges
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on area to remove small noisy regions
    min_contour_area = 100
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_contour_area]

    # Draw contours on the original image
    cv2.drawContours(image, large_contours, -1, (0, 255, 0), 2)

    # Display the resulting image with detected obstacles and features
    cv2.imshow("Obstacle Detection", image)

    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    # Break the loop if 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
camera.close()
cv2.destroyAllWindows()
