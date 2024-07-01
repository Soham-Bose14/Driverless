import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
#from eufs_msgs.msg import car_state
from ackermann_msgs.msg import AckermannDriveStamped
from eufs_msgs.msg import ConeArrayWithCovariance
import numpy as np
import pid
import STANLEY
#import stanley3
import time
import vref_using_waypoints as wp
import random
import math
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# import csv
# import atexit
# import os


#more accurate to take actual time interval
#rather than previously chosen timestamp
#But it increased the error even more, so better to take exact timestamp of 1/30 only

pid_obj = pid.pid_control()
stanley_obj = STANLEY.stanley() 
timestamp = 1/30
max_vel = 5.0
wp_count = 0
prev_error = 0
vref = 5.0
new_waypoints = []
added_wps = 0
blue_found = 0
yellow_found = 0
Xblue = []
Yblue = []
Xyellow = []
Yyellow = []

class car_state(Node):
    def __init__(self):
        super().__init__('CarHandler')
        self.subscription = self.create_subscription(Odometry,'/ground_truth/odom',self.send_car_input,10)
        self.subscription = self.create_subscription(ConeArrayWithCovariance, '/ground_truth/cones', self.get_cone_position,10)
        self.publisher = self.create_publisher(AckermannDriveStamped,'/cmd',10)

        '''self.fig = None
        self.ax = None
        self.scatter1 = None
        self.scatter2 = None'''


    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def send_car_input(self, msg):
        global max_vel
        global wp_count
        global prev_error
        global vref
        global Xyellow
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        yaw_x = msg.pose.pose.orientation.x
        yaw_y = msg.pose.pose.orientation.y
        yaw_z = msg.pose.pose.orientation.z
        yaw_w = msg.pose.pose.orientation.w

        '''#t0 = +2.0 * (w * x + y * z)
        #t1 = +1.0 - 2.0 * (x * x + y * y)
        #roll_x = math.atan2(t0, t1)
    
        #t2 = +2.0 * (w * y - z * x)sww1
        #t2 = +1.0 if t2 > +1.0 else t2
        #t2 = -1.0 if t2 < -1.0 else t2
        #pitch_y = math.asin(t2)
        #return yaw_z #roll_x,pitch_y,yaw_z

        t3 = +2.0 * (yaw_w * yaw_z + yaw_x * yaw_y)
        t4 = +1.0 - 2.0 * (yaw_y**2 + yaw_z**2)
        yawZ = np.arctan(t3/t4)
        self.yaw = yawZ'''

        a,b,c = self.euler_from_quaternion(yaw_x,yaw_y,yaw_z,yaw_w)
        self.yaw = c
        
        v_car = np.sqrt(np.square(vx) + np.square(vy) + np.square(vz))
        if (wp.crossed_waypoint(self.x, self.y, wp_count)):
            '''print('CROSSED WAYPOINT: ',end='')
            print(wp_count)'''
            vref = wp.get_ref_vel(self.x,self.y,max_vel,wp_count)
            wp_count = (wp_count+1)%38
        e = vref - v_car
        print('TARGET WAYPOINT: ',end='')
        print((wp_count)%38)
        # print('\nRef Vel: ',vref)
        # print('Car Vel: ',v_car)
        # print('x:',x)
        # print('y:',y)

        acc = pid_obj.required_acceleration(e, prev_error, timestamp)  
        steering = stanley_obj.calculate_steering(self.x,self.y,v_car,self.yaw,wp_count) 
        msg2 = AckermannDriveStamped()
        msg2.drive.acceleration = acc
        msg2.drive.steering_angle = steering
        self.publisher.publish(msg2)
        self.get_logger().info(f'Required Acceleration: {msg2.drive.acceleration} and Steering: {msg2.drive.steering_angle}')
        prev_error = e

    def to_global_frame(self,x,y,car_x,car_y,yaw):
        #xb = x*np.cos(yaw) + y*np.sin(yaw)
        #yb = y*np.cos(yaw) - x*np.sin(yaw)
        # x = x + car_x
        # y = y + car_y
        xg = x*np.cos(yaw) - y*np.sin(yaw) + car_x
        yg = x*np.sin(yaw) + y*np.cos(yaw) + car_y
        return xg,yg
    
    #The following doesn't work
    # def u_turn_problem(self,color1,color2,yaw):
    #     m = np.arctan(yaw)  #same slope for both reference lines
    #     c1 = color1[1] - m*color1[0]
    #     c2 = color2[1] - m*color2[0]
    #     d = (np.abs(c1-c2))/(np.sqrt(1 + m**2))
    #     if(d > 3.0):
    #         return True
    #     return False

    def u_turn_problem(self,color1,color2,yaw):
        #car's instantaneous trajectory: m*x - y -m*carx + cary = 0
        m = np.arctan(yaw)
        d1 = np.abs(m*color1[0] - color1[1] - m*self.x + self.y)/(np.sqrt(m**2 + 1))
        d2 = np.abs(m*color2[0] - color2[1] - m*self.x + self.y)/(np.sqrt(m**2 + 1))

        if(d1>4 or d2>4):
            return True
        return True

    def get_cone_position(self, msg):
        '''if self.fig is None or self.ax is None or self.scatter1 is None or self.scatter2 is None:
            self.initialize_plot()'''
        blue=[]
        yellow=[]
        points = []
        global new_waypoints
        global added_wps
        global blue_found
        global yellow_found
        count = added_wps

        global Xblue
        global Yblue
        global Xyellow
        global Yyellow
        blue_count = blue_found
        yellow_count = yellow_found
        
        for i in range(0,len(msg.blue_cones),2):
            x_blue,y_blue = msg.blue_cones[i].point.x, msg.blue_cones[i].point.y
            x_blue,y_blue = self.to_global_frame(x_blue,y_blue,self.x,self.y,self.yaw)
            #if(self.cone_is_good(x_blue,y_blue,Xblue,Yblue)):
            Xblue.append(x_blue)
            Yblue.append(y_blue)
            blue.append([x_blue,y_blue])
            points.append([x_blue,y_blue])


        for i in range(0,len(msg.yellow_cones),2):
            x_yellow,y_yellow = msg.yellow_cones[i].point.x, msg.yellow_cones[i].point.y
            x_yellow,y_yellow = self.to_global_frame(x_yellow,y_yellow,self.x,self.y,self.yaw)
            #if(self.cone_is_good(x_yellow,y_yellow,Xyellow,Yyellow)):
            Xyellow.append(x_yellow)
            Yyellow.append(y_yellow)
            yellow.append([x_yellow,y_yellow])
            points.append([x_yellow,y_yellow])

        blue_found = len(Xblue)
        yellow_found = len(Xyellow)
        '''print(f'Blue Cones: {blue}')
        print(f'Yellow Cones: {yellow}')'''
        #print(f'All cones: {points}')
        points = np.array(points)
        #print(type(points))
        #print(np.shape(points))

        with open('blue_cone_pos.csv', 'a') as fb:
            for i in range(blue_count, blue_found):
                #dist =math.sqrt(new_waypoints[i][0]**2+new_waypoints[i][1]**2)
                
                #print(f"X Distance from origin {new_waypoints[i][0] } anf y distance{new_waypoints[i][1]}")
                fb.write(f'{Xblue[i]},{Yblue[i]}')
                fb.write('\n')

        with open('yellow_cone_pos.csv', 'a') as fy:
            for i in range(yellow_count, yellow_found):
                #dist =math.sqrt(new_waypoints[i][0]**2+new_waypoints[i][1]**2)
                
                #print(f"X Distance from origin {new_waypoints[i][0] } anf y distance{new_waypoints[i][1]}")
                fy.write(f'{Xyellow[i]},{Yyellow[i]}')
                fy.write('\n')
                        
        try:
            tri = Delaunay(points)
            print(f'Points forming triangles: {tri.simplices}')


            for i in tri.simplices:
                array=[]
                if((i[0]<len(blue) and i[1]>=len(blue)) or (i[0]>=len(blue) and i[1]<len(blue))):
                    midpoint = [(points[i[0]][0]+points[i[1]][0])/2, (points[i[0]][1]+points[i[1]][1])/2]
                    possible = True
                    for j in new_waypoints:
                        if(np.sqrt((j[0]-midpoint[0])**2 + (j[1]-midpoint[1])**2) < 3.0):
                            possible = False
                    if((self.u_turn_problem(points[i[0]],points[i[1]],self.yaw))): 
                        possible = False
                    if(possible):
                        new_waypoints.append(midpoint)

                if((i[0]<len(blue) and i[2]>=len(blue)) or (i[0]>=len(blue) and i[2]<len(blue))):
                    midpoint = [(points[i[0]][0]+points[i[2]][0])/2, (points[i[0]][1]+points[i[2]][1])/2]
                    possible = True
                    for j in new_waypoints:
                        if((np.sqrt((j[0]-midpoint[0])**2 + (j[1]-midpoint[1])**2) < 3.0)):
                            possible = False
                    if(self.u_turn_problem(points[i[0]],points[i[2]],self.yaw)):
                        possible = False
                    if(possible):
                        new_waypoints.append(midpoint)
                    

                if((i[1]<len(blue) and i[2]>=len(blue)) or (i[1]>=len(blue) and i[2]<len(blue))):
                    midpoint = [(points[i[1]][0]+points[i[2]][0])/2, (points[i[1]][1]+points[i[2]][1])/2]
                    possible = True
                    for j in new_waypoints:
                        if((np.sqrt((j[0]-midpoint[0])**2 + (j[1]-midpoint[1])**2) < 3.0)):
                            possible = False
                    if(self.u_turn_problem(points[i[1]],points[i[2]],self.yaw)):
                       possible = False 
                    if(possible):
                        new_waypoints.append(midpoint) 
                    
                

        except Exception as e:
            print(f"An error occurred: {e}")
            return
        
        added_wps = len(new_waypoints)

        #print(f'New Waypoints: {new_waypoints}')

        with open('new_waypoints.csv', 'a') as f:
            for i in range(count, added_wps):
                #dist =math.sqrt(new_waypoints[i][0]**2+new_waypoints[i][1]**2)
                
                #print(f"X Distance from origin {new_waypoints[i][0] } anf y distance{new_waypoints[i][1]}")
                f.write(f'{new_waypoints[i][0]},{new_waypoints[i][1]}')
                f.write('\n')
 

        '''fig, ax = plt.subplots()
        xdata, ydata = zip(*new_waypoints)
        xdata = np.array(xdata)
        ydata = np.array(ydata)
        ln, = ax.plot([], [], 'ro')

        def init():
            ax.set_xlim(-50, 50)  # Adjust xlim based on the range of x coordinates
            ax.set_ylim(-50, 50)  # Adjust ylim based on the range of y coordinates
            return ln,

        def update(frame):
            # x = xdata[frame]
            # y = ydata[frame]
            x = xdata[int(frame)]
            y = ydata[int(frame)]
            ln.set_data(x, y)
            return ln,

        ani = FuncAnimation(fig, update, frames=np.linspace(0, len(new_waypoints)-1, 128),
                            init_func=init, blit=True)
        plt.pause(1)'''


        #self.update_plot()

    '''def cone_is_good(self,x,y,x_list,y_list):
        limit = 1.5
        for i in range(len(x_list)):
            if(np.sqrt((x-x_list[i])**2 + (y-y_list[i])**2) > limit):
                return False
        return True
    '''
        
    
            

        


def main(args=None):
    rclpy.init(args=args)
    my_car = car_state()
    start = time.time()
    while True:
        t = time.time()-start
        if(t >= timestamp):
           print(t)
           rclpy.spin_once(my_car)
           start = time.time()

    
    rclpy.shutdown()


if __name__=='__main__':
    main()