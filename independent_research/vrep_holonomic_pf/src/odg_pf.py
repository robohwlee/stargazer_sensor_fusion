#! /usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry           # robot pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from itertools import groupby

w_robot = 1.2                   # width of robot
gamma = 5
angle_range = 360
angle = 0

"""

LiDAR로부터 관측된 특정 time step에서의 data 값은 (dis, angle) 형태로 저장이 되어야 하는 것이 기본이다.

"""

# theta_goal을 받아줄 수 있어야 함으로 또 지정을 해줄 수 있는 코드를 짜야함
theta_goal = 0


# callback 함수
def rplidar_callback(msg):

    min_dis = 1e10

    for i in msg.ranges:
        if (i != 0) and (i < min_dis):
            min_dis = i 
        

    #print("minimum distace:", min_dis)

    sensor_data = []            # 전처리될 lidar data들                                  
    threshold_distance = 5                                                             
    threshold_continuity = 3    # number of continous elements                                                          
    filtered_sensor_data = []

    for angle, dis in enumerate(msg.ranges):
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

    #fancy print summary
    #print("[OBSTACLE INFO]")
    #for k, obs in enumerate(filtered_sensor_data):
    #    print("Obstacle No: {}".format(k+1))
    #    print("Obstacle Range: {} - {} degrees".format(obs[0][0], obs[-1][0]))
    #    print("-------------------------------")    
    #print()                           

    d_max = 20                                              # Hokuyo LiDAR의 최대 감지 거리 (20m라고 현재 가정)

    # POTENTIAL FIELD 구축

    
    total_field = 0
    total_field_list = []
    attractive_field = 0
    heading_angle = 0

    for theta_i in range(angle_range):

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
    #print(heading_angle)

    rotate(angle)

                                                            
def rotate(_angle):                                                                      
    #Starts a new node                                                             
    #rospy.init_node('head_angle', anonymous=True)                               
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()                                                              
                                                                                   
    # Receiveing the user's input                                                                                                             
    # clockwise = True                               
                                                                                   
    #Converting from angles to radians                                             
    angular_speed = 0                                          
    relative_angle = _angle                                          
                                                                                   
    #We wont use linear components                                                 
    vel_msg.linear.x = 0                                                             
    vel_msg.linear.y = 0                                                             
    vel_msg.linear.z = 0                                                             
    vel_msg.angular.x = 0                                                          
    vel_msg.angular.y = 0                                                          
                                                                                   
    # Checking if our movement is CW or CCW                                        
    if relative_angle > 0:                                                                  
        vel_msg.angular.z = -abs(angular_speed)                                    
    else:                                                                          
        vel_msg.angular.z = abs(angular_speed)                                     
    
    # Setting the current time for distance calculus                               
    t0 = rospy.Time.now().to_sec()                                                 
    current_angle = 0                                                              
                                                                                   
    while(current_angle < relative_angle):                                         
        velocity_publisher.publish(vel_msg)                                        
        t1 = rospy.Time.now().to_sec()                                             
        current_angle = angular_speed*(t1-t0)                                      
                                                                                   
                                                                                   
    #Forcing our robot to stop                                                     
    vel_msg.angular.z = 0                                                          
    velocity_publisher.publish(vel_msg)                                            
    #rospy.spin()
    


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
	X, Y, Z = quaternion_to_euler_angle(msg.orientation)
	print("Quaternoion ================")
	print("x : ", msg.orientation.x)
	print("y : ", msg.orientation.y)
	print("z : ", msg.orientation.z)
	print("w : ", msg.orientation.w)
	print("Euler -----------------------")
	print("X : ", X)
	print("Y : ", Y)
	print("Z : ", Z)


rospy.init_node("ODG-PF")
sub = rospy.Subscriber('/scan', LaserScan, rplidar_callback)
odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)
#rotate(angle)
rospy.spin()

