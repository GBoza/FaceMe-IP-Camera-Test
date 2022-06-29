import cv2

cam = cv2.VideoCapture(1)
s, img = cam.read()

if s:
  cv2.imwrite("imgdepth.jpg", img)

