#! /usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry           # robot pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from itertools import groupby
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
import time

w_robot = 1.2                   # width of robot
gamma = 5
angle_range = 180
angle = 0
threshold_distance = 4          # 난 2m 거리 이내에 있는 장애물에 대해서만 신경쓰겠다!   

"""
LiDAR로부터 관측된 특정 time step에서의 data 값은 (dis, angle) 형태로 저장이 되어야 하는 것이 기본이다.
"""

# theta_goal을 받아줄 수 있어야 함으로 또 지정을 해줄 수 있는 코드를 짜야함
theta_goal = 0

robot_angle = 0


# lidar callback 함수
def lidar_callback(msg):

    # for i in range(len(msg.ranges)):
    #     print(i+1, msg.ranges[i])


    # ============= 실시간 그래프 출력 ============= #
    """
    아직 원인을 찾지는 못했지만, v-rep 상에서는 분명 전방 180도만을 출력하게끔 설정했는데, 
    rostopic echo /scan을 통해 얻어지는 msg.ranges는 계속 360개의 data가 출력되고 분명 간격은 1도라고 함.
    그래서 현재로써는 일단 반토막해서 강제로 -90 ~ 90 도 범위로 맞춰져서 데이터 손실의 우려가 있긴 함.
    """
    
    plt.ion()

    plt.title('OD-MGPF', loc='center')
    plt.xlabel('LiDAR angle (degrees)')
    plt.ylabel('LiDAR measuremnts (m)')

    x_data = [ int(x/2)-90 for x in range(len(msg.ranges)) ]        # -90 ~ 90
    y_data = [ msg.ranges[179-y*2] for y in x_data ]
    
    x_data_1 = [ x for x in x_data if msg.ranges[179-x*2] > threshold_distance ]
    x_data_2 = [ x for x in x_data if msg.ranges[179-x*2] < threshold_distance ]
    x_data_3 = x_data

    y_data_1 = [ msg.ranges[179-y*2] for y in x_data_1 ]
    y_data_2 = [ msg.ranges[179-y*2] for y in x_data_2 ]
    y_data_3 = [ msg.ranges[179-y*2] if msg.ranges[179-y*2] <= threshold_distance else threshold_distance for y in x_data]
    y_data_4 = [ msg.ranges[179-y*2] if msg.ranges[179-y*2] > threshold_distance else threshold_distance for y in x_data]


    
    plt.axis([-90, 90, 0, 15])

    plt.figure(1)
    #plt.plot(x_data, y_data, color='b', label="LiDAR measurement")                 # orginal LiDAR 값을 출력하고 싶을 때
    #plt.plot(x_data_1, y_data_1, 'b', x_data_2, y_data_2, 'g')
    #plt.plot(x_data_2, y_data_2, color='g', label="obstacle")                       # 실질적으로 장애물로 취급되는 것만 출력
    
    # =============== fancy graph ===========================
    #plt.plot(x_data, y_data_3, color='g', label='obstacle range')  
    #plt.plot(x_data, y_data_4, color='b', label='else')  
    #plt.axhline(y=threshold_distance, color='r', label="threshold distance")
    #plt.draw()
    #plt.legend()
    #plt.pause(0.0001)
    #plt.clf()
    # ========================================== #


    min_dis = 1e10

    for i in y_data:
        if (i != 0) and (i < min_dis):
            min_dis = i 
        

    #print("minimum distace:", min_dis)

    sensor_data = []            # 전처리될 lidar data들                                                                                            
    threshold_continuity = 3    # number of continous elements                                                          
    filtered_sensor_data = []

    for angle, dis in zip(x_data, y_data):
        #print("angle: {}, dis: {}".format(angle, dis))
        if 0 < dis <= threshold_distance:
            sensor_data.append( (angle, dis) )
        else:
            sensor_data.append( (0, 0) )

        #print("sensor_data: ", sensor_data)
                                                                                   
    while True:                                                                        
        obs = []                                                                       
        for data in sensor_data:                       # (angle, dis) 형태의 tuple                                                                                       
            if data[1] > 0:                                                 
                obs.append(data)                                                          
                sensor_data = sensor_data[1:]                                                                                   
            else:                                                                                                                     
                sensor_data = sensor_data[1:]                                                          
                break                                                                  
                                                                                    
        if (len(obs) != 0) and (len(obs) > threshold_continuity) :                     
            filtered_sensor_data.append(obs)                                           
                                                                                    
        if len(sensor_data) == 0:                                                              
            break                                                                      

                                                                        
    #print("recognized obstacle:", filtered_sensor_data) 

    # ===== fancy print summary =====
    #print("[OBSTACLE INFO]")
    #for k, obs in enumerate(filtered_sensor_data):
    #    print("Obstacle No: {}".format(k+1))
    #    print("Obstacle Range: {} - {} degrees".format(obs[0][0], obs[-1][0]))
    #    print("-------------------------------")
    #    print()               

    d_max = 10                                              # Hokuyo LiDAR의 최대 감지 거리: 10m


    # ============================================================================================================================== #
    # ===================================================== POTENTIAL FIELD 구축 ===================================================== #

    total_field = 0
    total_field_list = []
    attractive_field = 0
    heading_angle = 0

    for theta_i in x_data:

        repulsive_field = 0

        for obs in filtered_sensor_data:                    # for each obstacle in filtered_sensor_data, (for every kth obstacles, )
            _sum = 0
            for i in obs:
                _sum += i[1]
                                                                                                                                
            theta_k = obs[int(len(obs) / 2)][0]             # center angle of each obstacle

            d_k = _sum / len(obs)                           # average distance for each obstacles          
            pi_k = len(obs) / 180 * math.pi                 # occupying angle for each obstacles

            sigma_k = math.atan2(d_k * math.tan(pi_k/2) + w_robot / 2, d_k)             # w_robot / 2를 해줘서 장애물의 크기를 조금씩 키움 (2보다 작아도 됨)
            #print(sigma_k)
            PI_k = 2 * sigma_k              # each obstacle's occupying range given from average distance and angle

            # repulsive field (Gaussian likelihood function)
            _d_k = d_max - d_k * d_max
            A_k = _d_k * math.exp(0.5)

            repulsive_field += A_k * math.exp( -((theta_k-theta_i) / 180 * math.pi)**2 / (2 * pow(sigma_k, 2) ) ) # 이 부분하고 위에 sigma_k 구하는 부분 이상
        
        attractive_field = gamma * abs( (theta_goal - theta_i) / 180 * math.pi )

        total_field = attractive_field + repulsive_field

        #print(attractive_field, repulsive_field)

        total_field_list.append( (total_field, theta_i) )

    #print(total_field_list)

    
    #print(total_field_list)
    min_total_field = [total_field_list[0]]
    
    #print(total_field_list)

    for pf, angle in total_field_list:    
        
        if pf < min_total_field[0][0]:
            min_total_field.pop()
            min_total_field.append((pf, angle))

    heading_angle = min_total_field[0][1] / 180 * math.pi 
    angle = heading_angle       

    rotate(angle)

    # ============================================================================================================================== #
    # ============================================================================================================================== #


def quaternion_to_euler_angle(msg):
	x = msg.x
	y = msg.y
	z = msg.z
	w = msg.w

	ysqr = y * y
	
	t0 = +2.0 * (w * x + y * z)
	t1 = +1.0 - 2.0 * (x * x + ysqr)
	X = math.degrees(math.atan2(t0, t1))
	
	t2 = +2.0 * (w * y - z * x)
	t2 = +1.0 if t2 > +1.0 else t2
	t2 = -1.0 if t2 < -1.0 else t2
	Y = math.degrees(math.asin(t2))
	
	t3 = +2.0 * (w * z + x * y)
	t4 = +1.0 - 2.0 * (ysqr + z * z)
	Z = math.degrees(math.atan2(t3, t4))
	
	return X, Y, Z

def odom_callback(msg):
    X, Y, Z = quaternion_to_euler_angle(msg.pose.pose.orientation)
    # print("Quaternoion ================")
    # print("x : ", msg.pose.pose.orientation.x)
    # print("y : ", msg.pose.pose.orientation.y)
    # print("z : ", msg.pose.pose.orientation.z)
    # print("w : ", msg.pose.pose.orientation.w)
    # print("Euler -----------------------")
    # print("X : ", X)
    # print("Y : ", Y)
    # print("Z : ", Z)
    global robot_angle 
    robot_angle = Z

#print(desired_angle)
                    
                                                            
def rotate(_angle):  

    # 이 _angle이 들어올 때 이미 rad 단위로 잘 들어오고 있음
    #print(_angle)

    #Starts a new node                                                             
    #rospy.init_node('head_angle', anonymous=True)                               
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
                                                                                   
    # Receiveing the user's input                                                                                                             
    # clockwise = True                               
                                                                                   
    #Converting from angles to radians                                             
    angular_speed = 0                               
    desired_angle = _angle                                          
                                                                                   
    #We wont use linear components                                                 
    vel_msg.linear.x = 0                                                             
    vel_msg.linear.y = 0                                                             
    vel_msg.linear.z = 0                                                             
    vel_msg.angular.x = 0                                                          
    vel_msg.angular.y = 0                                                          
                                                                                   
    # Checking if our movement is CW or CCW                                        
    if desired_angle > 0:                                                                  
        vel_msg.angular.z = -abs(angular_speed)                                    
    else:                                                                          
        vel_msg.angular.z = abs(angular_speed)                                     
    
    # Setting the current time for distance calculus                               
    t0 = rospy.Time.now().to_sec()  

    current_angle = robot_angle

    #print("Before starting....")
    #print("[current angle]: {}".format(current_angle)) 
    #print("[desired angle]: {}".format(desired_angle))                                                            
                                                                                   
    #if abs(current_angle - desired_angle) > 0.2:  
        #print("1") 
        #print("Updating....")
        #print("[current angle]: {}".format(current_angle)) 
        #print("[desired angle]: {}".format(desired_angle))                                           
    velocity_publisher.publish(vel_msg)                                        
    t1 = rospy.Time.now().to_sec()                                             
    current_angle += angular_speed*(t1-t0)

        #print(vel_msg)
    #else:
        #print("0")
        #Forcing our robot to stop
        # vel_msg.linear.x = 0                                                             
        # vel_msg.linear.y = 0                                                             
        # vel_msg.linear.z = 0                                                             
        # vel_msg.angular.x = 0                                                          
        # vel_msg.angular.y = 0   
        # vel_msg.angular.z = 0                                                          
        # velocity_publisher.publish(vel_msg)  

    
    rospy.sleep(0.01)

                                           
    
    


#rospy.init_node("ODG-PF")
sub = rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1, buff_size= 2**24)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
#rotate(angle)
rospy.spin()

