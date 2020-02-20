# stargazer_sensor_fusion
localization using sensor fusion with IMU, odom and stargazer

Author: Y.B.KIM
Linux Version: 18.04.4 LTS (Bionic Beaver)
ROS Version: Melodic

[ 터틀봇3를 이용한 제어 ]

    1.터틀봇 준비 
        1) 로봇 Raspberry Pi에는 무선충전기를 통해 전원 공급
        2) OpenCR에는 내장형 드론 배터리를 통해 전원 공급

    2.Robot BringUp 
        # 보다 구체적인 정보는 emanual.robotis.com의 [ ROS ] Bringup을 참고
        1) [ Remote PC ]
            -Ctrl + Alt + T를 통해 새로운 터미널 창을 생성한다.
                ~$ roscore        
        2) [ TurtleBot ]
            - ssh 통신을 통해 Turtlebot의 computer에 원격 접속
                ~$ ssh pi@172.24.191.99 # 공D617 기준
            -Ctrl + Alt + T를 통해 새로운 터미널 창을 생성한다.
                ~$ roslaunch turtlebot3_bringup turtlebot3_robot.launch

    3.Rviz 실행 
        1) [ Remote PC ]
            -Ctrl + Shift + T를 통해 다른 터미널 창을 옆에 생성한다.
                ~$ export TURTLEBOT3_MODEL = burger # 우리 로봇은 burger이므로 
                ~$ roslaunch turtlebot3_bringup turtlebot3_remote.launch
        2) [ Remote PC ]
            -Ctrl + Shift + T를 통해 다른 터미널 창을 옆에 생성한다.    
                ~$ rosrun rviz rviz -d rospack find turtlebot3_description /rviz/model.rviz
    
    4.robot_localization node 실행
        1) EKF
            ~$ roslaunch robot_localization ekf_template.launch
