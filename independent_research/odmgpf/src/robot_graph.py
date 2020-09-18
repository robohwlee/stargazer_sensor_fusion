#! /usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.transforms as mtransforms
import matplotlib.patches as mpatches
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import sys, os, time
from api import vrep
from datetime import datetime
import odmgpf
import pandas as pd
from regions import PixCoord, RectanglePixelRegion
from astropy.coordinates import Angle
import random
import scipy
import scipy.stats as stats
import csv

# EXPERIMENT PARAMETER
d_threshold = int(sys.argv[1]) # 논문 기준 2m
gamma = int(sys.argv[2]) # 논문 기준 (4~6)  
k_holo = int(sys.argv[3]) # holonomic 운동 모델 보정 변수

button_1 = False
button_3 = False
button_4 = False
button_5 = False


lidar = []

euler_z = 0

switch = 0
d_switch = 0

class Map:

    def __init__(self):
        try:
            vrep.simxFinish(-1)
        except:
            pass
        
        self.clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5) # Connect to V-REP
        #vrep.simxSynchronous(self.clientID, True)

        
        self.opmode_blocking = vrep.simx_opmode_blocking
        self.obs_handle1 = vrep.simxGetObjectHandle(self.clientID, objectName="obs1", operationMode=vrep.simx_opmode_blocking)[1]
        self.obs_handle2 = vrep.simxGetObjectHandle(self.clientID, objectName="obs2", operationMode=vrep.simx_opmode_blocking)[1]
        self.obs_handle3 = vrep.simxGetObjectHandle(self.clientID, objectName="obs3", operationMode=vrep.simx_opmode_blocking)[1]
        self.obs_handle4 = vrep.simxGetObjectHandle(self.clientID, objectName="obs4", operationMode=vrep.simx_opmode_blocking)[1]
        self.obs_handle5 = vrep.simxGetObjectHandle(self.clientID, objectName="obs5", operationMode=vrep.simx_opmode_blocking)[1]
        self.d_obs_handle = vrep.simxGetObjectHandle(self.clientID, objectName="Obstacle_shape_1", operationMode=vrep.simx_opmode_blocking)[1]
        self.robot_handle = vrep.simxGetObjectHandle(self.clientID, objectName="dualarm_mobile", operationMode=vrep.simx_opmode_blocking)[1]
        



        self.x_rand1 = 1.2362  #random.uniform(-2.5, 2.5)
        self.x_rand3 = -2.388  #random.uniform(-2.5, 2.5)
        self.x_rand4 =  1.8112  #random.uniform(-2.5, 2.5)
        self.x_rand5 = -2.5138  #random.uniform(-2.5, 2.5)

        vrep.simxSetObjectPosition(self.clientID, self.obs_handle1, -1, [6.75, self.x_rand1, 0.5], self.opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.obs_handle3, -1, [1.75, self.x_rand3, 0.5], self.opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.obs_handle4, -1, [-2.75, self.x_rand4, 0.5], self.opmode_blocking)
        vrep.simxSetObjectPosition(self.clientID, self.obs_handle5, -1, [-5.75, self.x_rand5, 0.5], self.opmode_blocking)
        

        self.alpha = 0.1

        self.deviation = 0

        self._pos_x = -8
        self._pos_y = 0

        self._d_x = 4.95
        self._d_y = -1.0696
      
        self.graph_switch = True

        self.csv_container = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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

        return X, Y, Z


    def calc_dis(self, x1, y1, x2, y2):
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
        
    
    def gradient(self, x):
        temp = 4.731 * math.exp(1.3*(x-1)) + 91.269 

        if temp > 255:
            return 255

        elif temp < 0:
            return 0

        return temp
        
    def myhook(self):
        print("[INFO] DONE")


    def odom_callback(self, msg):

        global switch

        global euler_z

        global button_1, button_3, button_4, button_5

        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        pos_z = msg.pose.pose.position.z
        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w

        if (abs(pos_y) > self.deviation):
            self.deviation = abs(pos_y)

        euler_x, euler_y, euler_z = self.quaternion_to_euler_angle(ori_x, ori_y, ori_z, ori_w)

        
        if self.graph_switch == True:

            plt.rcParams["figure.figsize"] = [9,9]
            plt.rcParams['legend.numpoints'] = 1
            plt.grid(alpha=0.2, color='k', linestyle='-.', dash_capstyle='butt', drawstyle='steps', linewidth=0.5)

            if switch == 0:

                plt.title('Obstacle Avoidance', loc='center')
                plt.xlabel('x-axis')
                plt.ylabel('y-axis')

            # ROBOT POSITION
            rp_x, rp_y, rp_z = vrep.simxGetObjectPosition(self.clientID, self.robot_handle, -1, vrep.simx_opmode_streaming)[1]
            
            # DYNAMIC OBSTACLE
            d_x, d_y, d_z = vrep.simxGetObjectPosition(self.clientID, self.d_obs_handle, -1, vrep.simx_opmode_streaming)[1]

            scale = self.calc_dis(d_x, d_y, rp_x, rp_y)
            
            #print(self.gradient(scale))

            #print(scale)

            if scale <= 10: 
                self.alpha = scale / 10
            else:
                self.alpha = 1.0

            # calculate distance between robot and obstacle

            if (scale >= 1):
                color = [ str(self.gradient(scale)) ]
                _color = (self.gradient(scale)/255, self.gradient(scale)/255, self.gradient(scale)/255)
            else:
                color = [255] 
                _color = (1, 1, 1)


            if (d_x != 0 and d_y != 0):

                # # 장애물과 로봇 사이의 거리
                # dis = self.calc_dis(rp_x, rp_y, 4.95, d_y) 
                
                # if (dis <= 1.1):
                #     coll_check1 = 'r'
                #     coll_check2 = (1,0,0)
                # else:
                #     coll_check1 = 'k'
                #     coll_check2 = (0,0,0)

                # DYNAMIC OBSTACLE
                #plt.scatter(d_y, d_x, marker='o', s=300, edgecolors='k', facecolor='none', label='dynamic obstacle')
                
                # [1] DYNAMIC OBSTACLE SHAPE
                #plt.scatter(-d_y, d_x, marker='o', s=600, edgecolors=coll_check1, facecolor=_color, label='dynamic obstacle')
                # circle = plt.Circle((-d_y, d_x), 0.5, edgecolor=coll_check1, facecolor=_color)
                # plt.gca().add_patch(circle)

                # self.csv_container[3] = -d_y
                # self.csv_container[4] = d_x

                # # [2] DYNAMIC OBSTACLE TRACE
                # if (d_y > self._d_y):
                #     dynamic_obstacle, = plt.plot( [-self._d_y, -d_y], [4.95, 4.95], marker=(3, 0, 90), markersize = 5, linewidth=1.0, color='b', label='dynamic obstacle')
                # else:
                #     dynamic_obstacle, = plt.plot( [-self._d_y, -d_y], [4.95, 4.95], marker=(3, 0, -90), markersize = 5, linewidth=1.0, color='b', label='dynamic obstacle')

                # self._d_y = d_y
                # self._d_x = d_x


                # ROBOT POSITION WITH TRACE
                x, y = -rp_y, rp_x #-pos_y, pos_x
                width, height = 0.6, 0.83
                angle = Angle(euler_z, 'deg') #Angle(euler_z, 'deg')

                center = PixCoord(x=x, y=y)
                reg = RectanglePixelRegion(center=center, width=width,
                                        height=height, angle=angle)
                patch = reg.as_artist(facecolor=_color, edgecolor='k', lw=1.0, fill=True)   #, label='Holonomic Robot')
                plt.gca().add_patch(patch)

                # [2] ROBOT CENTER & TRACE
                robot_footprint, = plt.plot( [-self._pos_y, -rp_y], [self._pos_x, rp_x], marker=(3, 0, euler_z), markersize = 5, linewidth=1.0, color='r', label='ODG-PF')
                self._pos_y = rp_y #pos_y             
                self._pos_x = rp_x #pos_x
                        

            dis1 = self.calc_dis(-self.x_rand1, 2.6, -rp_y, rp_x)
            dis3 = self.calc_dis(-self.x_rand3, -0.6, -rp_y, rp_x)
            dis4 = self.calc_dis(-self.x_rand4, -2.95, -rp_y, rp_x)
            dis5 = self.calc_dis(-self.x_rand5, -5.8, -rp_y, rp_x)

            # obs 1
            if (dis1 <= 1.2): 
                coll_check_1_1 = 'r'
                coll_check_1_2 = (1,0,0)
                button_1 = True
            else:
                coll_check_1_1 = 'k'
                coll_check_1_2 = (0,0,0)
                button_1 = False

            # obs 3
            if (dis3 <= 1.2): 
                coll_check_3_1 = 'r'
                coll_check_3_2 = (1,0,0)
                button_3 = True
            else:
                coll_check_3_1 = 'k'
                coll_check_3_2 = (0,0,0)
                button_3 = False

            # obs 4
            if (dis4 <= 1.2): 
                coll_check_4_1 = 'r'
                coll_check_4_2 = (1,0,0)
                button_4 = True
            else:
                coll_check_4_1 = 'k'
                coll_check_4_2 = (0,0,0)
                button_4 = False

            # obs 5
            if (dis5 <= 1.2): 
                coll_check_5_1 = 'r'
                coll_check_5_2 = (1,0,0)
                button_5 = True
            else:
                coll_check_5_1 = 'k'
                coll_check_5_2 = (0,0,0)
                button_5 = False
                

            # STATIC OBSTACLE 1
            if button_1 == False:
                center = PixCoord(x=-self.x_rand1, y = 6.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor='none', lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                button_1 = True

            else:
                center = PixCoord(x=-self.x_rand1, y=6.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor='none', lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                
            # STATIC OBSTACLE 3
            if button_3 == False:
                center = PixCoord(x=-self.x_rand3, y=1.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor='none', lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                button_3 = True

            else:
                center = PixCoord(x=-self.x_rand3, y=1.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor=coll_check_3_1, lw=1.0, fill=True)
                plt.gca().add_patch(patch)



            # STATIC OBSTACLE 4
            if button_4 == False:
                center = PixCoord(x=-self.x_rand4, y=-2.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor='none', lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                button_4 = True

            else:
                center = PixCoord(x=-self.x_rand4, y=-2.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor=coll_check_4_1, lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                
            # STATIC OBSTACLE 5
            if button_5 == False:
                center = PixCoord(x=-self.x_rand5, y=-5.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor='none', lw=1.0, fill=True)
                plt.gca().add_patch(patch)
                button_5 = True

            else:
                center = PixCoord(x=-self.x_rand5, y=-5.75)
                reg = RectanglePixelRegion(center=center, width=1,
                                        height=1)
                patch = reg.as_artist(facecolor='k', edgecolor=coll_check_5_1, lw=1.0, fill=True)
                plt.gca().add_patch(patch)                   
       
        
            #plt.grid()
            plt.xticks(np.arange(-10, 11, 1))
            plt.yticks(np.arange(-10, 11, 1))
            
            plt.draw()
            
            if switch < 2:
                try:
                    static_obstacle = mpatches.Patch(hatch='', color='black', label="static obstacle")
                    plt.legend(handles=[static_obstacle, dynamic_obstacle, robot_footprint])
                    switch += 1
                except:
                    static_obstacle = mpatches.Patch(hatch='', color='black', label="static obstacle")
                    plt.legend(handles=[static_obstacle])
            
            plt.pause(1)

            goal_dis = self.calc_dis(pos_x, pos_y, 9, 0) # V-REP 축이 뒤틀렸음

            simulationTime=vrep.simxGetLastCmdTime(self.clientID) / 1000 # sec로 단위 변환

            # f.close()

            if (goal_dis < 1.2 or simulationTime > 90):

                if (goal_dis) < 1.2:

                    x, y = -pos_y, pos_x
                    width, height = 0.6, 0.83
                    angle = Angle(euler_z, 'euler')                      # 원래는 Angle(euler_z, 'euler')였는데, 이렇게 바꾸면 인터넷 연결이 필요하던 Angle이 필요가 없어지게됨

                    center = PixCoord(x=x, y=y)
                    reg = RectanglePixelRegion(center=center, width=width,
                                            height=height, angle=angle)
                    patch = reg.as_artist(facecolor=_color, edgecolor='m', lw=1.5, fill=True, label='Goal Arrived')
                    plt.gca().add_patch(patch)


                print("[INFO] EPISODE DONE")
                plt.savefig('/home/yu-bin/ODMGPF_EXPERIMENT/OD-MGPF_{}_{}_{}_{:.3f}.png'.format(d_threshold, gamma, k_holo, self.deviation, dpi = 300))
                
                # RESULT TO CSV
                result = {'RESULT': [str(d_threshold)+" "+str(gamma)+" "+str(k_holo)], 'DEVIATION': [str(self.deviation)]}
                df = pd.DataFrame(result)

                if not os.path.exists('/home/yu-bin/ODMGPF_EXPERIMENT/output.csv'):
                    df.to_csv('/home/yu-bin/ODMGPF_EXPERIMENT/output.csv', index=False, mode='w', encoding='utf-8-sig')
                else:
                    df.to_csv('/home/yu-bin/ODMGPF_EXPERIMENT/output.csv', index=False, mode='a', encoding='utf-8-sig', header=False)
                rospy.signal_shutdown(self.myhook())
        
        

if __name__ == "__main__":

    rospy.init_node('polar_graph', anonymous=True)

    # 인스턴스 생성
    map = Map()
    
    odom_sub = rospy.Subscriber('/odom', Odometry, map.odom_callback, queue_size = 1, buff_size = 2**30)

    rospy.spin()

