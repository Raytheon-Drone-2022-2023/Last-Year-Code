import cv2
import time

capture = cv2.VideoCapture(0)
width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))

writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (width, height))

while True:
  success, img = capture.read()
  writer.write(img)
  cv2.imshow("Result", img)
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

capture.release()
writer.release()
cv2.destroyAllWindows()
