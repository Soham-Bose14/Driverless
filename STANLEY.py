import pandas as pd
import numpy as np
import math


#Stanley implemented taking front axle as ref
dataframe = pd.read_csv("/home/sohambose/ws/waypoints.csv")
X = list(dataframe.iloc[:,0])
Y = list(dataframe.iloc[:,1])

class stanley:
    e = 0 #Crosstrack error
    L = 1.580 #Chasis length
    k = 1.25
    ks = 0.5
    kh = 0.1
    max_steer = np.pi/4

    def to_body_frame(self,x,y,car_x,car_y,yaw):
        x = x-car_x
        y = y-car_y
        xb = x*np.cos(yaw) + y*np.sin(yaw)
        yb = y*np.cos(yaw) - x*np.sin(yaw)
        # xb = x*np.cos(yaw) - y*np.sin(yaw)
        # yb = y*np.cos(yaw) + x*np.sin(yaw)
        return xb,yb

    def calculate_steering(self,car_x,car_y,car_vel,yaw,wp_count):
        #Taking coordinates of the next and the previous waypoint in terms of body frame

        try:
            x1 = X[(wp_count-1)%38]
            delx = X[(wp_count)%38]-x1
            y1 = Y[(wp_count-1)%38]
            dely = Y[(wp_count)%38]-y1

        except:
            delx = X[0]-X[37]
            dely = Y[0]-Y[37]
            x1 = X[37]
            y1 = Y[37]
        
        self.e = (dely*(car_x-x1) - delx*(car_y-y1))/((dely**2 + delx**2)**0.5)
        #self.e = (dely*(0-x1) - delx*(0-y1))/((dely**2 + delx**2)**0.5)
        print('\nCrosstrack error: ',self.e)
        
        theta = np.arctan2(self.k*self.e,car_vel+self.ks)

        try:
            x1,y1 = self.to_body_frame(X[(wp_count-1)%38],Y[(wp_count-1)%38],car_x,car_y,yaw)
            x2,y2 = self.to_body_frame(X[(wp_count)%38],Y[(wp_count)%38],car_x,car_y,yaw)
             

        except:
            x1,y1 = self.to_body_frame(X[37],Y[37],car_x,car_y,yaw)
            x2,y2 = self.to_body_frame(X[0],Y[0],car_x,car_y,yaw)

        '''x_front = car_x + (self.L/2)*np.cos(yaw)
        y_front = car_y + (self.L/2)*np.sin(yaw)'''

        dely = y2-y1
        delx = x2-x1

        req_heading = np.arctan2(dely,delx)
        # heading_error = req_heading-yaw
        heading_error = req_heading

        delta = heading_error + self.kh*theta

        if(delta>self.max_steer):
            delta = self.max_steer
        elif(delta<(-1)*self.max_steer):
            delta = (-1)*self.max_steer

        print('Delta',delta)

        

        return delta

    

    
        
    