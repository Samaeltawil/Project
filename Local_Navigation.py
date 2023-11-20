def button_center():
    global state
    if button_center == 1:
        state = 1 if state==0 else 0

def obs_avoid(prox_horizontal, y):
    
    # ANN Obstacle Avoidance Approach
    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

    # Scale factors for sensors and constant factor
    sensor_scale = 200
    constant_scale = 20
    
    x = [0,0,0,0,0,0,0,0,0]
    
    if state != 0:
        # Memory
        x[7] = y[0]//10
        x[8] = y[1]//10
        
        for i in range(7):
            # Get and scale inputs
            x[i] = prox_horizontal[i] // sensor_scale
        
        y = [0,0]    
        
        for i in range(len(x)):    
            # Compute outputs of neurons and set motor powers
            y[0] = y[0] + x[i] * w_l[i]
            y[1] = y[1] + x[i] * w_r[i]
    else: 
        # In case we would like to stop the robot
        y = [0,0] 
    
    # Set motor powers
    motor_left_target = y[0]
    motor_right_target = y[1]