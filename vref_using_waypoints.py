import pandas as pd 
import numpy as np
import math
import STANLEY

st = STANLEY.stanley()
dataframe = pd.read_csv("/home/sohambose/ws/waypoints.csv")
#X = list(dataframe.iloc[:,0])
X = list(dataframe.iloc[:,0])
#print(type(X))
#print(X)

Y = list(dataframe.iloc[:,1])
#print(type(Y))
#print(Y)
prev_sign = 0
vmin = 3.0

def crossed_waypoint(car_x, car_y, wp_count):
    global prev_sign
    if(wp_count%38==0):
        delx = X[0]-X[37]
        dely = Y[0]-Y[37]
    else:
        delx = X[(wp_count)%38]-X[(wp_count-1)%38]
        dely = Y[(wp_count)%38]-Y[(wp_count-1)%38]
    #Slope of perpendicular to the line joining the next two waypoints passing through the immediate waypoint = -delx/dely
    m = -delx/dely
        
    #Eqn of perpendicular: mx-y-mx1+y1=0
    sign = m*car_x - car_y - m*X[wp_count%38] + Y[wp_count%38]
    
    if(prev_sign==0):
        prev_sign = sign

    if(sign*prev_sign<0):
        # print('\nCROSSED WAYPOINT:',end=' ')
        # print(wp_count)
        '''print('TARGET WAYPOINT: ',end='')
        print((wp_count+1)%38)'''
        prev_sign = sign
        return True
    return False



def get_ref_vel(car_x, car_y, max_car_vel, wp_count):
    weights = [0.6,0.3,0.1]
    theta = 0.0
    for i in range(3):
        delx = X[(wp_count+i)%38]-car_x
        dely = Y[(wp_count+i)%38]-car_y 
        theta += (np.arctan2(dely,delx))*weights[i]
    # print('Theta: ', end='')
    # print(theta,end=' ')
    vref = max_car_vel*((np.abs(np.cos(theta))))
    if(vref<vmin):
        vref = vmin

    # if np.isnan(vref):
    #     vref = 5.0 

    # print('Vref: ', end='')
    # print(vref)
    # print()
    return vref
    
# def get_ref_vel(car_x, car_y, max_car_vel,yaw, wp_count):
#     wmean_x = 0
#     wmean_y = 0
#     weights = [5,3,1]
#     for i in range(3):
#         #x,y = st.to_body_frame(X[(wp_count+i)%38],Y[(wp_count+i)%38],car_x,car_y,yaw)
#         wmean_x += weights[i]*X[(wp_count+i)%38]/9
#         wmean_y += weights[i]*Y[(wp_count+i)%38]/9
#     #theta = np.arctan2((wmean_y-car_y),(wmean_x-car_x))
#     theta = np.arctan2((wmean_y-car_y),(wmean_x-car_x))
#     print('Theta: ', end='')
#     print(theta,end=' ')
#     vref = max_car_vel*((np.abs(np.cos(theta)))**(1/3))
#     print('Vref: ', end='')
#     print(vref)
#     print()
#     return vref

        

