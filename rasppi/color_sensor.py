import cv2

import vision_pb2

def get_color(frame):
  print("Getting color")

  blur = cv2.blur(frame, (5, 5))
  lab = cv2.cvtColor(blur, cv2.COLOR_BGR2LAB)
  frame_l,frame_a,frame_b = cv2.split(lab)
  color = lab.mean(axis=0).mean(axis=0)
  a = int(color[1])
  b = int(color[2])

  red = (range(130, 200), range(140, 200))
  yellow = (range(110, 130), range(130, 180))
  green = (range(90, 110), range(120, 140))
  blue = (range(110, 130), range(50, 130))

  color = vision_pb2.InfoUpdate.Color.UNDEFINED
  color_str = "undefined"
  if a in red[0] and b in red[1]:
    color = vision_pb2.InfoUpdate.Color.RED
    color_str = "red"
  if a in yellow[0] and b in yellow[1]:
    color = vision_pb2.InfoUpdate.Color.YELLOW
    color_str = "yellow"
  if a in green[0] and b in green[1]:
    color = vision_pb2.InfoUpdate.Color.GREEN
    color_str = "green"
  if a in blue[0] and b in blue[1]:
    color = vision_pb2.InfoUpdate.Color.BLUE
    color_str = "blue"

  print("Found color", color_str)

  return color
