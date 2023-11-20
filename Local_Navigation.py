Up_threshold = 3500
Low_threshold = 2300

def state_switch(state, dist):
    if state == 1: # We're in Global Navigation
        if (dist[0] > Up_threshold) or (dist[1] > Up_threshold) or (dist[2] > Up_threshold) or (dist[3] > Up_threshold) or (dist[4] > Up_threshold):
            state = 2 # Switch to Local Navigation to avoid obtstacle. If obstacle is detected at any of the prox sensors.
            
    elif state == 2: # We're in Local Navigation
        if (dist[0] < Low_threshold) and (dist[1] < Low_threshold) and (dist[2] < Low_threshold) and (dist[3] < Low_threshold) and (dist[4] < Low_threshold):
            state = 1 # Go back to Global Navigation. If obstacle is far wrt to all prox sensors.
            
    return state        
            

def obs_avoid(prox_horizontal, y):
    
    # ANN Obstacle Avoidance Approach
    w_l = [40,  20, -20, -20, -40,  30, -10, 8, 0]
    w_r = [-40, -20, -20,  20,  40, -10, 30, 0, 8]

    # Scale factors for sensors and constant factor
    sensor_scale = 500
    constant_scale = 20
    
    x = [0,0,0,0,0,0,0,0,0]
    
    if state != 0:
        # Memory
        x[7] = y[0]//constant_scale
        x[8] = y[1]//constant_scale
        
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
    
    return y