import cv2
import numpy as np

UP_VAL = 70                                   # max value for the range of color considered as black
KERNEL = 5                                    # kernel size for the meddian blur 
OBST_THRESHOLD = 127                          # value for the thresholding, every value below that is put to 0, every value above is put to 255   
GOAL_THRESHOLD = 0.7                          # threshold for a high enough matching of the goal
ROBOT_THRESHOLD = 0.8                         # threshold for a high enough matching of the robot
TEMPLATE_ROBOT = "img/template_robot.png"     # source image for the template of the robot
TEMPLATE_GOAL = "img/template_goal.png"       # source image for the template of the goal

# initialize the template for the robot and the goal using two images
def defTemplate(s):
    # definition and treatment of the robot template
    template_robot = cv2.imread(TEMPLATE_ROBOT)
    template_robot = cv2.cvtColor(template_robot, cv2.COLOR_BGR2GRAY)
    template_robot = cv2.resize(template_robot, (s, s))
    # definition and treatment of the goal template
    template_goal = cv2.imread(TEMPLATE_GOAL)
    template_goal = cv2.cvtColor(template_goal, cv2.COLOR_BGR2GRAY)
    template_goal = cv2.resize(template_goal, (s, s))
    return [template_robot, template_goal]

# find obstacle using range and thresholding
def findObstacle(image):
    # define the range of value that are considered as black color
    lower_range = np.array([0, 0, 0], dtype = "uint8")
    upper_range = np.array([UP_VAL, UP_VAL, UP_VAL], dtype = "uint8")
    mask = cv2.inRange(image, lower_range, upper_range)
    # filtering
    filtered_img = cv2.medianBlur(mask, KERNEL) 
    # thresholding
    ret, image_obst = cv2.threshold(filtered_img, OBST_THRESHOLD, 255, 0)  
    return image_obst

# find goal using matchTemplate
def findGoal(image, template):
    w, h  = template.shape[::-1]
    # change to grayscale in order to use matchTemplate
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    # test if the matching is high enough
    if max_val > GOAL_THRESHOLD:
        found = True
        # get the middle of the template
        center = (max_loc[0] + w//2, max_loc[1] + h//2)
        return [center, found]
    else: 
        found = False
        return [[], found]

# track the robot using matchTemplate
def robotTracking(image, template):
    w, h  = template.shape[::-1]
    # change to grayscale in order to use matchTemplate
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    res = cv2.matchTemplate(image_gray, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    # test if the matching is high enough
    if max_val > ROBOT_THRESHOLD:
        found = True
        # get the middle of the template
        center = (max_loc[0] + w//2, max_loc[1] + h//2)
        return [center, found]
    else: 
        found = False
        return [(0, 0), found]
