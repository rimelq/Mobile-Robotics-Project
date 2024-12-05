import numpy as np
import math
import time

# constants 
MAX_DELTA_TIME = 1000                  # to prevent problem when using time.time() 
AVERAGE_LOOP_TIME = 0.02               # in case time.time() is above MAX_DELTA_TIME delta_time is set to AVERAGE_LOOP_TIME
SCALED_ANGULAR_SPEED = 0.000070        # scaling for the angular speed to take into account the distance between the camera and the plan
SCALED_SPEED = 0.6                     # scaling for the speed to take into account the distance between the camera and the plan
TIME_TO_GET = 0.05                     # time to get the speed value from the encoders 
MIN_DIST = 0.01                        # min distance that the robot need to have covered in order to have a new command

# proportional integrator controler parameters
MAX_INT = 100
KP = 200
KI = 0.2

# set the motors speed to the desired one
def motors(left, right):
    return {
        "motor.left.target": [left],
        "motor.right.target": [right],
    }

# compute the enclidian distance between two points
def eucl_distance(x1, y1, x2, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5

def stop_robot(node):
    node.send_set_variables(motors(0, 0))

# get the speed of the robot using the encoders
async def get_speed(node,client):
    await node.wait_for_variables({"motor.left.speed", "motor.right.speed"})
    await client.sleep(TIME_TO_GET)
    left_speed, right_speed = node.v.motor.left.speed, node.v.motor.right.speed
    return left_speed, right_speed 

# convert the speed from the encoder and compute the relative variation of position
def convert(left_speed, right_speed,time_s,scale):
    angular_speed_cst = SCALED_ANGULAR_SPEED * scale
    angular_speed = (right_speed - left_speed) * angular_speed_cst
    delta_angle = angular_speed * time_s                              # angle in radians
    delta_x =  SCALED_SPEED * scale * np.cos(delta_angle) * time_s         
    delta_y = SCALED_SPEED * scale * np.sin(delta_angle) * time_s  
    return delta_x, delta_y,delta_angle

# convert relative coordinates of the thymio to absolute cartesian coordinates
def to_cartesian_base(x, y, x_old, y_old, pos_x, pos_y):
    B = np.array([x_old, y_old])   
    A = np.array([x, y])
    C = np.array([pos_x, pos_y])
    AB = A - B
    v2 = AB / np.linalg.norm(AB)
    v1 = np.array([v2[1], -v2[0]])
    abs_pos = np.array([np.dot(C, v2), np.dot(C, v1)])
    real_val = np.add(abs_pos, A)
    return real_val[0], real_val[1]

###### Robot Class
class Robot:
    def __init__(self,dist_target,speed):
        self.DIST_TARGET_CTRL = dist_target
        self.SPEED = speed
        # position variables
        self.x = 0  
        self.y = 0
        self.x_old = 0
        self.y_old = 0
        # encoder
        self.x_enc = 0  
        self.y_enc = 0
        self.x_enc_old = 0
        self.y_enc_old = 0
        # camera
        self.x_cam = 0  
        self.y_cam = 0
        # variance of the estimate position of the thymio 
        self.var_x = 0
        self.var_y = 0
        
        self.old_err = 0

        # list of the points that the robot needs to go to
        self.targets = []
        self.target_Id = 1
        self.found = False
        
        self.last_move = 0
        self.start_move_old = 0
        self.start_move  = 0

        # starting time of the kidnapping
        self.kid_start = 0
        # starting time of the local obstacle avoidance
        self.avoid_time = 0

    # convert cartesian coordinates to thymio relative coordinates in order to get the direction of the current target
    def thymio_base(self):
        B = np.array([self.x_old, self.y_old])   
        A = np.array([self.x, self.y])  
        C = np.array(self.targets[self.target_Id]) 
        
        AB = A - B
    
        v2 = AB / np.linalg.norm(AB) 
        v1 = np.array([-v2[1], v2[0]]) 
        C_prime = C - A
        new_C = np.array([np.dot(C_prime, v1), np.dot(C_prime, v2)])
        return new_C

    # proportionnal integrator controler 
    def pi(self, err):
        self.old_err =  err
        integrale = 0
        if abs(integrale) <= MAX_INT:
            integrale += err
            
        u = KP * err + KI * integrale 
        return u

    # compute the command to give to the motor in order to reach the current target
    def control(self):
        target_bar = self.thymio_base()
        if target_bar[0] == 0:
            err = 0
        else:
            err = abs(math.atan(target_bar[0]/target_bar[1])) * target_bar[0] / abs(target_bar[0])
        diff_speed = self.pi(err)  
        return diff_speed

    # give command to the robot to make it go to its current target
    def move_robot(self, node):
        # check if the robot has move sufficiently to get a new command
        if (eucl_distance(self.x, self.y, self.x_old, self.y_old) > MIN_DIST):
            dist_from_target = eucl_distance(self.targets[self.target_Id][0], self.targets[self.target_Id][1], self.x, self.y)

            # check if the robot is close enough to its target
            if dist_from_target <= self.DIST_TARGET_CTRL: 
                self.target_Id  += 1
                if(self.target_Id == len(self.targets)): # return True is last target reached
                    return True

            # get the command 
            direction = self.control()
            node.send_set_variables(motors(self.SPEED + int(direction) , self.SPEED - int(direction)))

            # time variable for the computation of the encoder position
            self.last_move = time.time()
            self.start_move_old = self.start_move
            self.start_move = time.time()

            # new coordinates
            self.x_old = self.x
            self.y_old = self.y
        return False

    
    async def pos_encodeur(self, node, client, scale):
        delta_time = self.last_move - self.start_move_old
        if delta_time > MAX_DELTA_TIME:
            delta_time = AVERAGE_LOOP_TIME
        x,y = self.x_enc,self.y_enc
        x_old,y_old = self.x_enc_old, self.y_enc_old
        self.x_enc_old, self.y_enc_old = self.x_enc, self.y_enc
        delta_x, delta_y, delta_angle = 0, 0, 0
        if  (x_old == x) and (y_old == y):
            print("X_old = x et y_old = y pour les encodeurs dans pos_encodeur")
            return x + 1, y + 1
        left_speed, right_speed = await get_speed(node,client)                                 # get the speed
        delta_x, delta_y,delta_angle = convert(left_speed, right_speed, delta_time, scale)        # get the delta speed
        pos_x, pos_y = to_cartesian_base(x, y, x_old, y_old, delta_x, delta_y)
        return pos_x, pos_y
    