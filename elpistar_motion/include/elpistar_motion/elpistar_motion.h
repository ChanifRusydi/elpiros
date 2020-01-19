#ifndef ELPISTAR_MOTION_CONTROLLERS_H
#define ELPISTAR_MOTION_CONTROLLERS_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>

#define WALK_FREQUENCY (15)
#define WALK 0
#define FRONT_STANDUP 1
#define BACK_STANDUP 2
 
class ElpistarMotionController{
 private:
 
  //ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  //ROS Topic Publisher
  ros::Publisher goal_joint_states_pub_;
  //Elpistar Motion Controller Parameter

  std::string robot_name_;

  public:
   ElpistarMotionController();
   ~ElpistarMotionController();
   void walk(int step);
   void stop();
   void front_standup();
   void back_standup();
   
  private:
   void initPublisher();
   void motion(uint8_t type, uint8_t pn);
};

#endif
