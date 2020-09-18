from api import vrep
import sys
import time
import math


try:
    vrep.simxFinish(-1) # just in case, close all opened connections
except:
    pass


clientID=vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
if clientID !=-1:
    print("Connected to remote API server")

    vrep.simxStopSimulation(clientID, operationMode=vrep.simx_opmode_blocking)

    _,handle=vrep.simxGetObjectHandle(clientID,'dualarm_mobile',vrep.simx_opmode_blocking)
    time.sleep(1)

    vrep.simxSetObjectPosition(clientID, handle,-1,[0.0,0.0,0.1],vrep.simx_opmode_blocking)
    time.sleep(1)

    vrep.simxSetObjectOrientation(clientID,handle,-1,[math.pi,math.pi/2, 0.0],vrep.simx_opmode_blocking)
    time.sleep(1)

    #vrep.simxFinish(clientID)
    #time.sleep(1)


    #vrep.simxPauseSimulation(clientID, operationMode=vrep.simx_opmode_oneshot)
    
    vrep.simxStartSimulation(clientID, operationMode=vrep.simx_opmode_blocking)
    vrep.simxSynchronousTrigger(clientID)

else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")




