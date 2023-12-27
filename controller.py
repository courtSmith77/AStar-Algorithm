import numpy as np
import matplotlib.pyplot as plt

def motion_model(state, control) :

    x_0 = state[0]
    y_0 = state[1]
    h_0 = state[2]

    dt = control[0]
    v = control[1]
    w = control[2]

    x = x_0 + v*np.cos(h_0 + w*dt)*dt + np.random.normal(0,0.001)
    y = y_0 + v*np.sin(h_0 + w*dt)*dt + np.random.normal(0,0.001)
    h = h_0 + w*dt + np.random.normal(0,0.001)

    return x,y,h

def check_accel(prev_vel, calc_vel, dt) :
    
    v_accel = (calc_vel[0]-prev_vel[0])/dt
    w_accel = (calc_vel[1]-prev_vel[1])/dt

    if (abs(v_accel > 0.288)) and (v_accel > 0):
        v = prev_vel[0] + 0.288*dt
    elif (abs(v_accel > 0.288)) and (v_accel < 0):
        v = prev_vel[0] - 0.288*dt
    else :
        v = calc_vel[0]

    if (abs(w_accel > 5.579)) and (w_accel > 0):
        w = prev_vel[1] + 5.579*dt
    elif (abs(v_accel > 5.579)) and (v_accel < 0):
        w = prev_vel[1] - 5.579*dt
    else :
        w = calc_vel[1]
    
    return v,w 

class Controller():

    def __init__(self, cols_x, rows_y, start) :

        self.path_y = rows_y
        self.path_x = cols_x

        self.current_x = start[0]
        self.current_y = start[1]
        self.current_h = 0

        self.control_path_x = [start[0]]
        self.control_path_y = [start[1]]
        self.control_path_h = [0]

        self.current_v = 0
        self.current_w = 0

        self.Kp_v = 0.005
        self.Kp_w = 0.11
        self.Ki_v = 0.0000005
        self.Ki_w = 0.000001
        self.error_sum_v = 0.0
        self.error_sum_w = 0.0
        self.max_output = 10000000000.0
        self.min_output = 0.0

        self.dt = 0.1
        self.tolerance = 0.01
        
    def control_path(self):

        breaking = False
        for pp in range(len(self.path_x)):

            waypoint_x = self.path_x[-1*(pp+1)]
            waypoint_y = self.path_y[-1*(pp+1)]

            curr_x = self.current_x
            curr_y = self.current_y

            dist_error = np.sqrt((waypoint_x-curr_x)**2 + (waypoint_y-curr_y)**2)
            ang_error = np.arctan2((waypoint_y-curr_y),(waypoint_x-curr_x)) - self.current_h
            arctan_org = np.arctan2((waypoint_y-curr_y),(waypoint_x-curr_x))

            Kp_v = self.Kp_v
            Kp_w = self.Kp_w
            Ki_v = self.Ki_v
            Ki_w = self.Ki_w

            self.count = 0

            while dist_error > self.tolerance :

                dist_error = np.sqrt((waypoint_x-self.current_x)**2 + (waypoint_y-self.current_y)**2)
                ang_error = np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) - self.current_h
                atan_e = np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x))

                if (abs(arctan_org) - np.pi) < 0.1 and (abs(arctan_org) - np.pi) > -0.1:
                    quadrant_h = (self.current_h % (2*np.pi))
                    if quadrant_h < 0 :
                        quadrant_h += 2*np.pi
                    
                    if quadrant_h > 0.0 and quadrant_h < np.pi :
                        if np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) > 0 :
                            ang_error = np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) - quadrant_h
                        else :
                            ang_error = (np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) + 2*np.pi) - quadrant_h
                
                # determine error for integral control
                self.error_sum_v += dist_error
                self.error_sum_w += ang_error

                # determine v and w values from PI control
                v_temp = (Kp_v*dist_error + Ki_v*self.error_sum_v)/self.dt
                w_temp = (Kp_w*ang_error + Ki_w*self.error_sum_w)/self.dt

                # check if accelerationsare greater than they should be
                # find true v and w values
                self.current_v, self.current_w = check_accel([self.current_v, self.current_w], [v_temp, w_temp], self.dt)

                # run the motion model to move the robot using v and w
                self.current_x, self.current_y, self.current_h = motion_model([self.current_x, self.current_y, self.current_h], [self.dt, self.current_v, self.current_w])

                self.control_path_h.append(self.current_h)
                self.control_path_x.append(self.current_x)
                self.control_path_y.append(self.current_y)

                self.count += 1

                if (dist_error > 1.0):
                    print("Ending path early")
                    breaking = True
                    break

            if breaking :
                break

    def plotting(self, fig, ax) :

        ax.plot(self.control_path_x, self.control_path_y)
        for pp in range(len(self.control_path_h)) : 

            if (pp % 1) == 0 :
                angle = self.control_path_h[pp]
                x = self.control_path_x[pp]
                y = self.control_path_y[pp]

                ddx = 0.25*np.cos(angle)
                ddy = 0.25*np.sin(angle)

                ax.arrow(x,y,ddx,ddy,color="orange")


class Controller_Live():

    def __init__(self, start) :

        self.current_x = start[0]
        self.current_y = start[1]
        self.current_h = 0.0

        self.control_path_x = [start[0]]
        self.control_path_y = [start[1]]
        self.control_path_h = [0]

        self.current_v = 0
        self.current_w = 0

        self.Kp_v = 0.005
        self.Kp_w = 0.11
        self.Ki_v = 0.0000005
        self.Ki_w = 0.000001
        self.error_sum_v = 0.0
        self.error_sum_w = 0.0
        self.max_output = 10000000000.0
        self.min_output = 0.0

        self.dt = 0.1
        self.tolerance = 0.01

    def control_planning(self, goal) :

        waypoint_x = goal[0]
        waypoint_y = goal[1]

        curr_x = self.current_x
        curr_y = self.current_y

        dist_error = np.sqrt((waypoint_x-curr_x)**2 + (waypoint_y-curr_y)**2)
        ang_error = np.arctan2((waypoint_y-curr_y),(waypoint_x-curr_x)) - self.current_h
        arctan_org = np.arctan2((waypoint_y-curr_y),(waypoint_x-curr_x))

        Kp_v = self.Kp_v
        Kp_w = self.Kp_w
        Ki_v = self.Ki_v
        Ki_w = self.Ki_w

        self.count = 0

        while dist_error > self.tolerance :

            dist_error = np.sqrt((waypoint_x-self.current_x)**2 + (waypoint_y-self.current_y)**2)
            ang_error = np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) - self.current_h

            if (abs(arctan_org) - np.pi) < 0.1 and (abs(arctan_org) - np.pi) > -0.1:
                quadrant_h = (self.current_h % (2*np.pi))
                if quadrant_h < 0 :
                    quadrant_h += 2*np.pi
                
                if quadrant_h > 0.0 and quadrant_h < np.pi :
                    if np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) > 0 :
                        ang_error = np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) - quadrant_h
                    else :
                        ang_error = (np.arctan2((waypoint_y-self.current_y),(waypoint_x-self.current_x)) + 2*np.pi) - quadrant_h
            
            # determine error for integral control
            self.error_sum_v += dist_error
            self.error_sum_w += ang_error

            # determine v and w values from PI control
            v_temp = (Kp_v*dist_error + Ki_v*self.error_sum_v)/self.dt
            w_temp = (Kp_w*ang_error + Ki_w*self.error_sum_w)/self.dt

            # check if accelerationsare greater than they should be
            # find true v and w values
            self.current_v, self.current_w = check_accel([self.current_v, self.current_w], [v_temp, w_temp], self.dt)

            # run the motion model to move the robot using v and w
            self.current_x, self.current_y, self.current_h = motion_model([self.current_x, self.current_y, self.current_h], [self.dt, self.current_v, self.current_w])

            self.control_path_h.append(self.current_h)
            self.control_path_x.append(self.current_x)
            self.control_path_y.append(self.current_y)

            self.count += 1

            if (dist_error > 1.0):
                print("Ending path early")
                break

        return self.current_x, self.current_y


