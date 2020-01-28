#ifndef ELPISTAR_MOTION_CONTROLLERS_H
#define ELPISTAR_MOTION_CONTROLLERS_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <sensor_msgs/JointState.h>
#include <elpistar_imu/EulerIMU.h>
#include <elpistar_msgs/DXLServer.h>

#define WALK_FREQUENCY (15)
#define WALK 0
#define FRONT_STANDUP 1
#define BACK_STANDUP 2
#define CONTROL 3

typedef struct PID{
   float Kp,Ki,Kd,Ts;
   float P,I,D;
   float error,last_error,SP;
   float u;
   bool en;
   bool walk_r;
};
class ElpistarMotionController{
 private:
 
  //ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  //ROS Topic Publisher and Subscriber
  ros::Publisher goal_joint_states_pub_;
  ros::Subscriber position_sub_;

  //ROS Service Client
  ros::ServiceClient move_dxl_client_;
  //Elpistar Motion Controller Parameter
  int robot_y;
  std::string robot_name_;

  public:
   ElpistarMotionController();
   ~ElpistarMotionController();
   void walk(int step);
   void walk_ready();
   void stop();
   void front_standup();
   void back_standup();
   void euler_pos_cb(const elpistar_imu::EulerIMU::ConstPtr &msg);   
  private:
   void initPublisher();
   void initSubscriber();
   void initClient();
   PID phi_ctrl;
   void motion(uint8_t type, uint8_t pn);
};

#endif
