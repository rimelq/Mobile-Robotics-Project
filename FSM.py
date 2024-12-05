from utils import *
from computerVision import *
from globalNav import *
from tdmclient import ClientAsync, aw
from control import *
from kalmanFilter import *
from localNav import *

## constants 
# variances of the encoders and the camera
VAR_ENC = 0.05
VAR_CAM = 0.01
# factors for the global navigation
SENSITIVITY_FACTOR = 0.5
SMOOTHING_FACTOR = 1.5

# ground proximity sensor threshold for kidnapping
PROX_THRESHOLD = 50

verbose = False  

# state = 0, initialization phase, the program scan the image, looks for the robot, the goal and the obstacles
def globalPlanning(image, thymio, SCALING_FACTOR, template_rbt, template_goal):
    obst_list = []
    [(thymio.x, thymio.y), thymio.found] = robotTracking(image, template_rbt) # search the robot
    (thymio.x_cam, thymio.y_cam) = (thymio.x, thymio.y)                       # assign the coordinates of the robot as the ones found
                                                                              # by the camera (as the encoders don't work yet)
    [goal, found_g] = findGoal(image, template_goal)                          # search the goal
    # print informations 
    if verbose:                                                                
        if thymio.found:
            print("Robot found")
        else:
            print("Robot not found")
        if found_g:
            print("Goal found")
        else:
            print("Goal not found")
            
    if thymio.found and found_g:                                              # we need the position of the robot and the goal
                                                                              # to find the shortest path
        start = (thymio.x, thymio.y)  
        # find the obstacles and look for the shortest path
        image_obst = findObstacle(image)
        thymio.targets, obst_list = find_shortest_path(image_obst, SCALING_FACTOR, start, goal, verbose) 
        thymio.target_Id = 1
        return [goal, obst_list]
    elif found_g:
        return [goal, []]
    else:
        return [[], []]

# state = 1, check the position of the robot acording to the camera and the encoders and apply a Kalman filter to estimate the position
async def robotMoving(thymio, image, template_rbt, node, client, SCALE_IMG):
     # Track the robot's position using vision
    [(thymio.x_cam, thymio.y_cam), thymio.found] = robotTracking(image, template_rbt)
    # apply the Kalman filter   
    thymio.x, thymio.var_x = kalman(thymio.x_enc, thymio.x_cam, thymio.var_x, VAR_CAM, VAR_ENC, thymio.found)
    thymio.y, thymio.var_y = kalman(thymio.y_enc, thymio.y_cam, thymio.var_y, VAR_CAM, VAR_ENC, thymio.found)
    thymio.x_enc, thymio.y_enc = thymio.x, thymio.y

    # make the robot move according to its position wrt its target position
    is_finished = thymio.move_robot(node)                                     
    
    thymio.x_enc, thymio.y_enc = await thymio.pos_encodeur(node, client, SCALE_IMG)

    return is_finished

# state = 2, local navigation 
async def localAvoidance(speed, node, aw, thymio, client, scale_img, state, image, template_robot, previous_speed=[0, 0]):
    # Track the robot's position using vision
    [(thymio.x_cam, thymio.y_cam), thymio.found] = robotTracking(image, template_robot)
    
    # time variable for the computation of the encoder position
    thymio.x, thymio.var_x = kalman(thymio.x_enc, thymio.x_cam, thymio.var_x, VAR_CAM, VAR_ENC, thymio.found)
    thymio.y, thymio.var_y = kalman(thymio.y_enc, thymio.y_cam, thymio.var_y, VAR_CAM, VAR_ENC, thymio.found)
    thymio.x_enc, thymio.y_enc = thymio.x, thymio.y
    
    # Read proximity sensor values
    prox_horizontal = read_prox_sensors(node, client, aw)

    # Compute motor speeds for obstacle avoidance
    motor_speeds = local_nav(
        sensitivity_factor=SENSITIVITY_FACTOR,
        smoothing_factor=SMOOTHING_FACTOR,
        prox_horizontal=prox_horizontal,
        desired_speed=[speed, speed],
        previous_speed=previous_speed
    )

    # Send motor commands to the robot
    node.send_set_variables(motors(motor_speeds[0], motor_speeds[1]))
    
    # time variable for the computation of the encoder position
    thymio.last_move = time.time()
    thymio.start_move_old = thymio.start_move
    thymio.start_move = time.time()
    
    # Update the robot's position using encoders
    thymio.x_enc, thymio.y_enc = await thymio.pos_encodeur(node, client, scale_img)

# check for local obstacle or kidnapping
def updateState(state, client, node, aw, thymio):
    # Read proximity sensor values
    prox_horizontal = read_prox_sensors(node, client, aw)
    obst = [prox_horizontal[i] for i in range(5)]  # Sensors 0 to 4

    # State 1: Goal Tracking
    if state == 1:
        # Check if any sensor detects an obstacle
        if any(obst[i] > OBST_THRESHOLD_HIGH for i in range(5)):
            state = 2  # Switch to Obstacle Avoidance

    # State 2: Obstacle Avoidance
    elif state == 2:
        # Check if all sensors are below the low threshold
        if all(obst[i] < OBST_THRESHOLD_LOW for i in range(5)):
            state = 4  # Switch back to Goal Tracking
            thymio.avoid_time = time.time()
            node.send_set_variables(motors(thymio.SPEED, thymio.SPEED))

    # Check for kidnapping
    if kidnappingDetection(node, client):
        stop_robot(node)
        state = 3  # Kidnapped state
        thymio.kid_start = time.time()

    return state

# kidnapping detection using ground proximity sensor, if the robot is lifted then it is kidnapped
def kidnappingDetection(node, client):
    aw(node.wait_for_variables({"prox.ground.delta"}))
    aw(client.sleep(0.01))
    prox = node.v.prox.ground.delta[1]
    if (prox < PROX_THRESHOLD):
        return True
    else:
        return False
