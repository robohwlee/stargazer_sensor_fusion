#! /usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from itertools import groupby
import numpy as np
import numpy.linalg as lin
import matplotlib.pyplot as plt
from matplotlib import animation
import time
import subprocess
import sys
import os

# ODMGPF 그래프
GRAPH1_ON = False
GRAPH2_ON = False


# EXPERIMENT PARAMETER
d_threshold = int(sys.argv[1]) # 논문 기준 2m
gamma = int(sys.argv[2]) # 논문 기준 (4~6)  
k_holo = int(sys.argv[3]) # holonomic 운동 모델 보정 변수
deviation = 0 # 실험 결과

w_robot = 0.83 # robot의 width는 0.61 x 0.83                   
angle_range = 180
angle = 0         
#theta_goal = 0
goal_count = 0
def sigmoid(x):
    return 1 / (1 + math.exp(-x))

def sign(x):
    if (x >= 0):
        return 1

    else:
        return -1


class ODMGPF:
    def __init__(self, gamma, d_threshold, k_holo):
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1, buff_size=2**30)
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1, buff_size=2**30)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1, buff_size = 2**10)
        self.x_pos = 0
        self.y_pos = 0
        self.count = 0
        self.MARKERS_MAX = 1
        self.odmgpf_angle = 0
        self.d_max = 10 # LiDAR 최대 감지 거리
        
        # EXPERIMENT PARAMETER
        self.gamma = gamma
        self.d_threshold = d_threshold
        self.k_holo = k_holo

        # theta_goal 계산을 위해서
        self.goal_info = {"pos_x": 10,"pos_y": 0,"pos_z": 0,"ori_x": 0,"ori_y": 0,"ori_z": 0,"ori_w": 0}
        self.robot_info = {"pos_x": -9,"pos_y": 0,"pos_z": 0,"ori_x": 0,"ori_y": 0,"ori_z": 0,"ori_w": 0}

        self.pos_x = 0
        self.pos_y = 0
        self.pos_z = 0
        self.ori_x = 0
        self.ori_y = 0
        self.ori_z = 0
        self.ori_w = 0

        # IMU
        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.quat_w = 0

        self.euler_x = 0
        self.euler_y = 0
        self.euler_z = 0
        

        self.robot_angle = 0
        self.theta_goal = 0 
        self.lidar_ranges = []
        self.delta_t = 0.05                    # 20 Hz


    def imu_callback(self, msg):

        self.quat_x = msg.orientation.x
        self.quat_y = msg.orientation.y
        self.quat_z = msg.orientation.z
        self.quat_w = msg.orientation.w


        self.euler_x, self.euler_y, self.euler_z = self.quaternion_to_euler_angle(self.quat_x, self.quat_y, self.quat_z, self.quat_w)
       

    def lidar_callback(self, msg):
        
        # DELTA TIME
        # self.delta_t = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9 - self.delta_t 

        # Lidar scan 값 정보

        self.lidar_ranges = []

        for i in msg.ranges[90:271]: # -90도에서 90도 사이의 정보만 | 총 181개의 각도 point에서 측정
            
            self.lidar_ranges.append(i)

        self.potential_field()


    def polar_graph(self):
    
        xs = np.linspace(0, 2*np.pi, 360)
        xs[359] = xs[0]
        _xs = [i for i in list(range(360))]
        ys = [self.lidar_ranges[j] for j in _xs]
        ys[359] = ys[0]

        ax = plt.subplot(111, polar = True)

        for x, y in zip(xs, ys):
            ax.plot(x, y, 'bo', ms=2)
            ax.set_theta_offset(math.pi * 1.5)
        #print(euler_z)

        if (0 <= self.odmgpf_angle and self.odmgpf_angle <= math.pi/2):
            angle = math.pi/2 + self.odmgpf_angle
        elif (-math.pi/2 <= self.odmgpf_angle and self.odmgpf_angle < 0):
            angle = -self.odmgpf_angle + math.pi

        plt.arrow(0, 0, 3 * math.cos(angle), 3 * math.sin(angle), alpha = 0.5, width = 0.01, edgecolor = 'red', facecolor = 'red', lw = 2, zorder = 5)

        ax.set_title("ODMGPF", va="bottom")
        plt.draw()
        plt.pause(0.000001)
        plt.clf()
        

    def potential_field(self):

        plt.rcParams["figure.figsize"] = (9,12)                         # graph 크기 고정

        plt.ion()                                                       # interactive mode on

        x_data = [ x for x in range(-90, 91) ]        # -90 ~ 90
        ops_x_data = [ -x for x in x_data ]
        y_data = [ self.lidar_ranges[i+90] for i in x_data ]
        
        y_data_3 = [ self.lidar_ranges[i+90] if self.lidar_ranges[i+90] <= self.d_threshold else self.d_threshold for i in x_data]
        y_data_4 = [ self.lidar_ranges[i+90] if self.lidar_ranges[i+90] >= self.d_threshold else self.d_threshold for i in x_data]
        
        plt.axis([-90, 90, 0, 30])

        if GRAPH1_ON == True:

            # =============== fancy graph 1 ===========================
            plt.subplot(211)

            plt.title('Obstacle Detection', loc='center')
            plt.xlabel('LiDAR angle (degrees)')
            plt.ylabel('LiDAR measuremnts (m)')
            
            plt.plot(ops_x_data, y_data_3, color='g', label='obstacle range')
            plt.plot(ops_x_data, y_data_4, color='b', label='else')
            plt.axhline(y=self.d_threshold, color='r', label="d_threshold")
            plt.xticks(np.arange(-90, 91, 10))
            plt.legend()

            # ========================================== #

        min_dis =  min([x for x in y_data if x > 0])        
        
        sensor_data = []            # 전처리될 lidar data들                                                                                            
        threshold_continuity = 3    # number of continous elements                                                          
        filtered_sensor_data = []

        for angle, dis in zip(x_data, y_data):
            if (0 < dis <= self.d_threshold):
                sensor_data.append( (angle, dis) )
            else:
                sensor_data.append( (0, 0) )
            
                                                                                    
        while True:                                                                        
            obs = []                                                                       
            for data in sensor_data: # (angle, dis) 형태의 tuple set                                                                                      
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

        # print("recognized obstacle:", filtered_sensor_data) 

        # #fancy print summary
        # print("[OBSTACLE INFO]")
        # for k, obs in enumerate(filtered_sensor_data):
        #    print("Obstacle No: {}".format(k+1))
        #    print("Obstacle Range: {} - {} degrees".format(obs[0][0], obs[-1][0]))
        #    print("-------------------------------")    
        # print()   

        # ============================================================================================================================== #
        # ===================================================== POTENTIAL FIELD 구축 ===================================================== #

        total_field = 0
        attractive_field = 0

        total_field_list = []
        attractive_field_list = []
        repulsive_field_list = []
        
        heading_angle = 0

        A_k = 0

        for theta_i in x_data:  # [degree]

            original_repulsive_field = 0
            min_repulsive_field = 0
            max_repulsive_field = 0

            for obs in filtered_sensor_data:                    # for each obstacle in filtered_sensor_data, (for every kth obstacles)
                
                min_dis = 11 # 최대 거리 20m를 초과하지 못하므로 21m로 초기화시켜놓기
                max_dis = -1 # 최소 거리 0m보다 작을 수 없으므로 0m로 초기화시켜놓기

                min_idx = 0
                max_idx = 0

                for idx, _obs in enumerate(obs): # 장애물로 인식된 entry 중에서 dis를 비교해서 해당 angle을 찾기
                    
                    # [1] obs를 순회하면서 가장 작은 dis와 해당 angle을 찾는 과정 
                    if _obs[1] < min_dis:
                        min_idx = idx
                        min_dis = _obs[1]

                    # [2] obs를 순회하면서 가장 큰 dis와 해당 angle을 찾는 과정 
                    if _obs[1] > max_dis:
                        max_idx = idx
                        max_dis = _obs[1]
                
                # 이제 min, max인 entry에 대한 정보를 얻었으니 본격적으로 G.M.M을 적용시켜보자. 

                _sum = 0
                for i in obs:
                    _sum += i[1]
                                                                                                                                    
                theta_k = obs[int(len(obs) / 2)][0]             # center angle of each obstacle [degree]
                min_theta_k = min_idx
                max_theta_k = max_idx

                d_k = _sum / len(obs)                           # average distance for each obstacles          
                min_d_k = min_dis
                max_d_k = max_dis

                pi_k = (obs[-1][0] - obs[0][0]) / 180 * math.pi                 # occupying angle for each obstacles [rad]
                min_pi_k = pi_k * min_d_k / d_k
                max_pi_k = pi_k * max_d_k / d_k
                
                
                sigma_k = math.atan2(d_k * math.tan(pi_k/2) + w_robot / 2, d_k)             # w_robot / 2를 해줘서 장애물의 크기를 조금씩 키움 (2보다 작아도 됨)
                min_sigma_k = math.atan2(min_d_k * math.tan(min_pi_k/2) + w_robot / 2, min_d_k)
                max_sigma_k = math.atan2(max_d_k * math.tan(max_pi_k/2) + w_robot / 2, max_d_k)
                
                #print(sigma_k)
                PI_k = 2 * sigma_k              # each obstacle's occupying range given from average distance and angle

                # repulsive field (Gaussian likelihood function)
                _d_k = self.d_max - d_k
                min__d_k = min_d_k
                max__d_k = self.d_max - max_d_k

                A_k = _d_k * math.exp(0.5)
                min_A_k = min__d_k * math.exp(0.5)
                max_A_k = max__d_k * math.exp(0.5)

                # [1] Min Repulsive Field
                min_repulsive_field = min_A_k * math.exp( -((theta_k-(theta_i-min_theta_k)) / 180 * math.pi)**2 / (2 * pow(min_sigma_k, 2) ) ) 

                # [2] Max Repulsive Field
                max_repulsive_field = max_A_k * math.exp( -((theta_k-(theta_i-max_theta_k)) / 180 * math.pi)**2 / (2 * pow(max_sigma_k, 2) ) ) 

                # [3] Original Repulsive Field
                original_repulsive_field = A_k * math.exp( -((theta_k-theta_i) / 180 * math.pi)**2 / (2 * pow(sigma_k, 2) ) ) 

            repulsive_field = 0.6 * original_repulsive_field + 0.2 * min_repulsive_field + 0.2 * max_repulsive_field
            
            attractive_field = self.gamma * abs( (self.theta_goal - theta_i) / 180 * math.pi )

            total_field = attractive_field + repulsive_field

            attractive_field_list.append( (attractive_field, theta_i) )
            repulsive_field_list.append( (repulsive_field, theta_i) )
            total_field_list.append( (total_field, theta_i) )


        min_total_field = [ total_field_list[0] ]
        
        for pf, angle in total_field_list:    
            
            if pf < min_total_field[0][0]:
                min_total_field.pop()
                min_total_field.append((pf, angle))

        heading_angle = min_total_field[0][1] / 180 * math.pi 

        angle = heading_angle

        self.odmgpf_angle = angle

        if GRAPH1_ON == True:

            # =============== fancy graph 2 ===========================
            
            plt.subplot(212)
            
            plt.title('Potential Field Flow', loc='center')
            plt.xlabel('LiDAR angle (degrees)')
            plt.ylabel('Potential Field')
            
            # potential field 
            plt.plot(x_data, [i[0] for i in total_field_list ], color='g', label='total field')
            plt.plot(x_data, [i[0] for i in attractive_field_list ], color='b', label='attractive field')
            plt.plot(x_data, [i[0] for i in repulsive_field_list ], color='r', label='repulsive field')

            # min total potential field
            plt.scatter([ i[1] for i in min_total_field ], [ i[0] for i in min_total_field ], s=80, facecolor='none',edgecolors='b', label='min total field')
            plt.xticks(np.arange(-90, 91, 10))
            plt.draw()
            plt.legend()
            plt.pause(0.0001)        
            
            plt.clf()

        attractive_field += 1e-10
        repulsive_field += 1e-10

        # print("=========================================")
        # print("[INFO] Potential Field")
        # print("[0] min LiDAR distance: {:.2f}".format(min_dis))
        # print("[1] total potential field: {:.2f}".format(total_field))
        # print("[2] attractive potential field: {:.2f}".format(attractive_field))
        # print("[3] repulsive potential field: {:.2f}".format(repulsive_field))
        # print("[4] heading angle: {:.2f} degrees".format(angle * 180 / math.pi))

        if GRAPH2_ON == True:
            self.polar_graph()
        
        odmgpf.behavior()


    def goal_callback(self, msg):

        global goal_count

        #if (goal_count == 0):
        
        # POSE INFO
        goal_pos_x = 7  #msg.pose.position.x
        goal_pos_y = 0  #msg.pose.position.y
        goal_pos_z = 0  #msg.pose.position.z

        goal_ori_x = 0  #msg.pose.orientation.x
        goal_ori_y = 0  #msg.pose.orientation.y
        goal_ori_z = 0  #msg.pose.orientation.z
        goal_ori_w = -1 #msg.pose.orientation.w

        # DICT INITIALIZE
        self.goal_info["pos_x"] = goal_pos_x
        self.goal_info["pos_y"] = goal_pos_y
        self.goal_info["pos_z"] = goal_pos_z

        self.goal_info["ori_x"] = goal_ori_x
        self.goal_info["ori_y"] = goal_ori_y
        self.goal_info["ori_z"] = goal_ori_z
        self.goal_info["ori_w"] = goal_ori_w



    def odom_callback(self, msg):

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        ori_x = self.quat_x
        ori_y = self.quat_y
        ori_z = self.quat_z
        ori_w = self.quat_w
        

        # DICT INITIALIZE
        self.robot_info["pos_x"] = pos_x
        self.robot_info["pos_y"] = pos_y
        self.robot_info["pos_z"] = pos_z

        self.robot_info["ori_x"] = ori_x
        self.robot_info["ori_y"] = ori_y
        self.robot_info["ori_z"] = ori_z
        self.robot_info["ori_w"] = ori_w

        #print("pos: ({:.3f},{:.3f},{:.3f}) | ori: ({:.3f},{:.3f},{:.3f},{:.3f})".format(self.pos_x, self.pos_y, self.pos_z, self.ori_x, self.ori_y, self.ori_z, self.ori_w))
        self.calc_angle()



    def calc_angle(self):

        d = math.sqrt(pow(self.robot_info["pos_x"]-self.goal_info["pos_x"],2) + pow(self.robot_info["pos_y"]-self.goal_info["pos_y"],2))
        d_x = self.robot_info["pos_x"]-self.goal_info["pos_x"]
        d_y = self.robot_info["pos_y"]-self.goal_info["pos_y"]
        _theta1 = math.atan(d_x/d_y) * 180 / math.pi     

        if _theta1 > 0:
            theta1 = 90 - _theta1
        else:
            theta1 = -(90 + _theta1)                         

        theta2 = self.euler_z

        # self.theta_goal
        #self.theta_goal = -(theta1-theta2) # 이렇게 계산할 줄 알았는데, 논문에서는 고정 값으로 두었다고 함 그냥 0으로 두자



    def quaternion_to_euler_angle(self, x, y, z, w):

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

        self.gen_marker(x, y, z, w)
        
        return X, Y, Z



    def gen_marker(self, x, y, z, w):
        # -------------------------------- RVIZ 상 MARKER PLOT -------------------------------------
        
        # [1] ROBOT HEADING MARKDER
        robot_marker = Marker()
        robot_marker.header.frame_id = "/base_footprint"
        robot_marker.type = robot_marker.ARROW
        robot_marker.action = robot_marker.ADD
        robot_marker.scale.x = -1
        robot_marker.scale.y = 0.2
        robot_marker.scale.z = 0.2
        robot_marker.color.a = 1.0
        robot_marker.color.r = 0.0
        robot_marker.color.g = 0.0
        robot_marker.color.b = 1.0
        robot_marker.pose.orientation.w = self.robot_angle
        robot_marker.pose.position.x = x
        robot_marker.pose.position.y = y 
        robot_marker.pose.position.z = z 

        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if(self.count > self.MARKERS_MAX):
            markerArray.markers.pop(0)

        markerArray.markers.append(robot_marker)

        # [2] OD-MGPF HEADING MARKDER
        odmgpf_marker = Marker()
        odmgpf_marker.header.frame_id = "/base_footprint"
        odmgpf_marker.type = odmgpf_marker.ARROW
        odmgpf_marker.action = odmgpf_marker.ADD
        odmgpf_marker.scale.x = 1
        odmgpf_marker.scale.y = 0.2
        odmgpf_marker.scale.z = 0.2
        odmgpf_marker.color.a = 1.0
        odmgpf_marker.color.r = 1.0
        odmgpf_marker.color.g = 0.0
        odmgpf_marker.color.b = 0.0
        odmgpf_marker.pose.orientation.w = self.theta_goal / 180 * math.pi
        odmgpf_marker.pose.position.x = x
        odmgpf_marker.pose.position.y = y 
        odmgpf_marker.pose.position.z = z 


        # We add the new marker to the MarkerArray, removing the oldest
        # marker from it when necessary
        if(self.count > self.MARKERS_MAX):
            markerArray.markers.pop(0)

        markerArray.markers.append(odmgpf_marker)

        # Renumber the marker IDs
        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1

        # Publish the MarkerArray
        publisher.publish(markerArray)

        self.count += 1

        # -----------------------------------------------------------------------



    def pub_vel_msg(self, lin_x, lin_y, lin_z, ang_x, ang_y, ang_z):
        
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10, tcp_nodelay=True)
        vel_msg = Twist()

        vel_msg.linear.x = lin_x
        vel_msg.linear.y = lin_y
        vel_msg.linear.z = lin_z

        vel_msg.angular.x = ang_x
        vel_msg.angular.y = ang_y
        vel_msg.angular.z = ang_z

        velocity_publisher.publish(vel_msg)


    
    def behavior(self):  

        """
        Control 논문의 수식을 정리해보자면, 
        
        u[k]  = (1 / ||a||_1) * u_ref (if ||a||_1 > 1)
              = u_ref (otherwise) 인데, 

        이때, 

        [1] v_max: 로봇이 이동할 수 있는 최대 속도
        [2] psi_ref: total potential field를 최소로 만들어 주는 robot의 heading angle
        [3] psi_k: 매 Time-step에서의 robot의 heading angle
        [4] delta_t: 매 lidar callback 함수 호출 시간 간격
        [5] B: 3차원 Rotation Matrix

        """
        
        # [1] Max Velocity (설정해주기 나름)
        v_max = 9.83 * 0.076 # m/s according to swedishWheel in V-REP
        w_max = v_max / (0.281+0.2355) # rad/s considering wheel seperation

        # [2] Ref Robot heading angle (feat. Potential Field)
        psi_ref = -self.odmgpf_angle # [rad] (논문에서는 우측을 +로 잡은 반면 실제 기준 축에서는 시계방향이 마이너스이므로)

        # [3] k_th Robot heading angle (feat. IMU)
        psi_k = self.euler_z / 180 * math.pi # [degree -> rad]

        # [4] DELTA_T
        # self.delta_t

        # [5]-1 행렬 B
        B = np.array(([math.cos(psi_k), -math.sin(psi_k), 0], [math.sin(psi_k), math.cos(psi_k), 0], [0, 0, 1]))

        # [5]-2 B의 역행렬
        B_inv = lin.inv(B)

        # U_REF

        temp = np.array(([v_max * math.cos(psi_ref), v_max * math.sin(psi_ref), (psi_ref - psi_k) / self.delta_t]))

        u_ref = np.matmul(B_inv, np.transpose(temp))

        # a
        a = np.array([u_ref[0]/v_max, u_ref[1]/v_max, u_ref[2]/w_max]) 
        L1_norm_a = abs(u_ref[0]/v_max) + abs(u_ref[1]/v_max) + abs(u_ref[2]/w_max)

        if (L1_norm_a > 1):
            u_k = u_ref / L1_norm_a
        else:
            u_k = u_ref 
       
        self.pub_vel_msg(u_k[0]*self.k_holo, u_k[1]*self.k_holo, 0, 0, 0, u_k[2])

        # print("============== HOLONOMIC ROBOT MODEL ===============")
        # print("v_max: {}".format(v_max))
        # print("w_max: {}".format(w_max))
        # print("psi_ref: {:.2f} | {}".format(psi_ref, psi_ref * 180 / math.pi))
        # print("psi_k: {:.2f} | {}".format(psi_k, psi_k * 180 / math.pi))
        # print("delta_t: {:.2f}".format(self.delta_t))
        # print("Matrix B")
        # for i in range(3):
        #     for j in range(3):
        #         print(B[i][j], end=" ")
        #     print()
        # print()

        # print("Matrix Inv B")
        # for i in range(3):
        #     for j in range(3):
        #         print(B_inv[i][j], end=" ")
        #     print()
        # print()

        # print("u_ref")
        # for k in range(3):
        #     print(u_ref[k], end=" ")
        # print()

        # print()

        # print("u_k")
        # for l in range(3):
        #     print(u_k[l], end=" ")
        # print()

        # print("L1_norm: {}".format(L1_norm_a))

        # print("publishing /cmd_vel as ...")
        # print("linear: ")
        # print("  x: {}".format(u_k[0]*self.k_holo))
        # print("  y: {}".format(u_k[1]*self.k_holo))
        # print("  z: {}".format(0))
        # print("angular: ")
        # print("  x: {}".format(0))
        # print("  y: {}".format(0))
        # print("  z: {}".format(u_k[2]))


        
if __name__ == '__main__':

    #subprocess.call("/home/yu-bin/reset.py", shell=True)

    time.sleep(3)
    
    markerArray = MarkerArray()
    publisher = rospy.Publisher('odmgpf_marker', MarkerArray, queue_size=10, tcp_nodelay=True)
    rospy.init_node('ODMGPF', anonymous=True)
    odmgpf = ODMGPF(gamma, d_threshold, k_holo)
    
    rospy.spin()