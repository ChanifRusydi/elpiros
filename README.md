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
Set the threshold in rqt_reconfigure, then save the value with update_vision script on robot:
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
Stop the robot service:
```
    sudo systemctl stop elpistar
```
launch all package in separate terminal, elpistar_motion and elpistar_imu in root (sudo su then source.bashrc)
```
  launch elpistar_motion with debug:=true
  launch elpistar_imu with debug_en:=true
  read the euler[3] value in elpistar_imu
  edit elpistar_motion.launch file
```
