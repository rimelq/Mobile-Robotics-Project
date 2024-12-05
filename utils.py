import cv2
from control import *

# draw circles at the robot position with several options;
# if draw_cam_enc is set as True the position estimated by the camera and the one estimated by the encoders will be 
# displayed in red respectively in yellow
# else only the position estimated after the Kalman filter will be displayed in green
def drawRobot(image, thymio, template_robot, draw_cam_enc = False):
    w, h  = template_robot.shape[::-1]
    if draw_cam_enc:
        cv2.circle(image, (int(thymio.x_enc), int(thymio.y_enc)), w//2, (0, 255, 255), 3)      # encoders position in yellow
        if thymio.found:
            cv2.circle(image, (int(thymio.x_cam), int(thymio.y_cam)), w//2, (0, 0, 155), 3)    # camera position in red
    else:
        cv2.circle(image, (int(thymio.x), int(thymio.y)), w//2, (0, 255, 0), 3)                # position with Kalman in green
    # draw the shortest path and a pink line between the robot and the current target
    if len(thymio.targets) != 0:
        cv2.line(image, (int(thymio.x), int(thymio.y)), thymio.targets[thymio.target_Id], (255, 10, 255), 4) 
        for i in range(len(thymio.targets)-1):
            cv2.line(image, thymio.targets[i], thymio.targets[i+1], (255, 255, 255), 4) 

# draw a blue circle around the goal
def drawGoal(image, goal, template_goal):
    if len(goal) != 0:
        w, h  = template_goal.shape[::-1]
        cv2.circle(image, goal, w//2, (255, 0, 100), 3)                                        # goal in blue

# draw the obstacles in orange 
def drawObstacle(image, obst_list):
    if len(obst_list) != 0:
        cv2.drawContours(image, obst_list, -1, (0,120,255), 3)                                 # obstacles in orange