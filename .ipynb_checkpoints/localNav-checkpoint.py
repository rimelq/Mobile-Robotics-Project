from control import motors


#Threshold to switch states
OBST_THRESHOLD_HIGH = 3500
OBST_THRESHOLD_LOW = 2200

def read_prox_sensors(node,client,aw):
    aw(node.wait_for_variables({"prox.horizontal"}))
    aw(client.sleep(0.01))
    prox = node.v.prox.horizontal
    return prox
    
# Smooths the transition between two speeds.
def smooth_speed(previous_speed, new_speed, smoothing_factor=0.9):
    return smoothing_factor * previous_speed + (1 - smoothing_factor) * new_speed

# Calculates motor speeds for obstacle avoidance using an ANN approach.
def local_nav(sensitivity_factor, smoothing_factor, prox_horizontal, desired_speed, previous_speed):
    # Weights for the neural network (left and right motors)
    weights_left = [-40, -20, -20, 20, 35, -10, 30, 0, 8]
    weights_right = [40, 20, -20, -20, -35, 30, -10, 8, 0]

    # Scaling factors
    SENSOR_SCALE = 800
    MEMORY_SCALE = 20

    # Initialize neural network inputs
    nn_inputs = [0] * 9

    # Memory inputs (scaled previous speeds)
    nn_inputs[7] = desired_speed[0] / MEMORY_SCALE
    nn_inputs[8] = desired_speed[1] / MEMORY_SCALE

    # Process proximity sensor inputs with sensitivity adjustment
    for i in range(7):
        nn_inputs[i] = (prox_horizontal[i] / SENSOR_SCALE) * sensitivity_factor

    # Compute motor outputs from ANN neurons
    motor_speeds = [0, 0]
    for i in range(len(nn_inputs)):
        motor_speeds[0] += nn_inputs[i] * weights_left[i]
        motor_speeds[1] += nn_inputs[i] * weights_right[i]

    # Smooth the motor speeds to avoid abrupt changes
    motor_speeds[0] = int(smooth_speed(previous_speed[0], motor_speeds[0], smoothing_factor))
    motor_speeds[1] = int(smooth_speed(previous_speed[1], motor_speeds[1], smoothing_factor))

    return motor_speeds
