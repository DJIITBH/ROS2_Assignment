import os
import cv2
print("Weights path exists:", os.path.exists('/home/vedh/workspaces/nav_ws/yolov3.weights'))
print("Config path exists:", os.path.exists('/home/vedh/workspaces/nav_ws/yolov3.cfg'))
print("Names path exists:", os.path.exists('/home/vedh/workspaces/nav_ws/coco.names'))
net = cv2.dnn.readNet('/home/vedh/workspaces/nav_ws/yolov3.weights','/home/vedh/workspaces/nav_ws/yolov3.cfg')
print("YOLO model loaded successfully:", not net.empty())