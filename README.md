Elpistar ROS package
============
ROS package for Elpistolero

Camera Tuning
-------------
Connect your robot with LAN
Stop the robot service:
```
    sudo systemctl stop elpistar
```
Reset camera and servo
Start service for tuner:
```
    sudo systemctl start elpistar_tuner
```
Open rqt_reconfigure and rqt_image_view in your PC:
```
    rosrun rqt_reconfigure rqt_reconfigure
    rqt_image_view
```
Set the threshold in rqt_reconfigure, then save the value with update_vision script:
    ```
    ./update_vision.sh
    ```
stop the tuner:
    ```
    sudo systemctl stop elpistar_tuner
    ```
reboot the robot

PID Tuning
----------
