cd elpistar_vision/params &&
export ROS_MASTER_URI=http://192.168.0.100:11311
rosrun dynamic_reconfigure dynparam dump /vision_controller vision.yaml
