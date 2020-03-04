# vrep_holonomic

ROS stargazer EKF(extended kalman filter) simulation package for turtle bot 3(burger)
will be updated for other mobile robots

<br/>

## Requirements

### ROS
http://wiki.ros.org/ROS/Installation

### Turtlebot3(burger used)
User should be able to handle the turtlebot according to the following manual
http://emanual.robotis.com/docs/en/platform/turtlebot3/setup/#setup

#### Power(Battery)
- LI-PO 11.1V 1800mAh LB-12 19.98Wh 5C battery for stargazer and OpenCR
- LI-PO battery or any portable powerbank is possible for Raspberry Pi

### v-rep(optional)
http://www.coppeliarobotics.com/downloads.html

### stargazer connection
#### ttyUSB port connection
Check the port number
```bash
dmesg |grep ttyUSB
```
ex) FTDI USB Serial Device conveter now attached to /dev/ttyUSBn (n: integer number)

#### Use minicom or other tools to change the port number
```bash
sudo install minicom
```
```bash
sudo minicom -s
```
Modify the port number as follows
```bash
serial port setup - A(Serial Device) - /dev/ttyUSBn (founded above)
```

#### Check and open the port
- Status(Time and ttyUSBn should be displayed on the terminal)
```bash
ls -l /dev/ttyUSBn
```
Also, the following message about dialout should be displayed
```bash
crw-rw---- 1 root dialout
```
If not, you should include user into 'dialout' group fisrt

- Authorization
```bash
chmod 777 /dev/ttyUSBn
```
Authorize the port to read(4), write(2) and execute(1)


<br/>

## Run

### Turtlebot
#### Network Configuration
- Remote PC
```bash
ROS_MASTER_URI = http://IP_OF_REMOTE_PC:11311
ROS_HOSTNAME = IP_OF_REMOTE_PC
```
- TurtleBot
```bash
ROS_MASTER_URI = http://IP_OF_REMOTE_PC:11311
ROS_HOSTNAME = IP_OF_TURTLEBOT
```

#### Remote PC
Remote control for the turtlebot using ssh connection (or directly connect to Pi)
```bash
roscore
```
```bash
ssh ${RASPBERRY_PI_NAME}@${REMOTE_IP}
```
specify which model will be used
```bash
export TURTLEBOT3_MODEL = burger
```

#### TurtleBot
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

#### Remote PC
- Bringup
```bash
roslaunch turtlebot3_bringup turtlebot3_remote.launch
```
- Rviz
```bash
rosrun rviz rviz -d rospack find turtlebot3_description /rviz/model.rviz
```
- Teleoperation
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

### Stargazer
```bash
roslaunch stargazer stargazer.launch
```

### EKF
```bash
roslaunch robot_localization ekf_template.launch
```