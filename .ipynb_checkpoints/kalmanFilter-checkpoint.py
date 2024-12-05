# we implement the Kalman filter to get an estimated position
def kalman(pos_enc, pos_cam, pos_var, var_cam, var_enc, found):
    if found:
        sum_var = pos_var + var_enc
        i_t = pos_cam - pos_enc
        k_t = sum_var/(sum_var + var_cam)
        new_pos = pos_enc + k_t * i_t
        new_var = (1 - k_t)*sum_var
    # if the camera cannot see the robot, the estimated position is only based on the encoders
    else:
        new_pos = pos_enc
        new_var = pos_var + var_enc
    return new_pos, new_var