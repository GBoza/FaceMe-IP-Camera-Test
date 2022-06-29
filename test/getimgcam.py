import cv2

cam = cv2.VideoCapture(0)
s, img = cam.read()

if s:
  cv2.imwrite("imgcam.jpg", img)

