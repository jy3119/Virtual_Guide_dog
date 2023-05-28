import cv2

camera = cv2.VideoCapture(0)

ret, image = camera.read()

cv2.imwrite("image.jpg", image)

cv2.imshow("Image", image)
cv2.waitKey(0)
cv2.destroyoAllWindows()

while True:

    ret, frame = camera.read()
    cv2.imshow("Frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()