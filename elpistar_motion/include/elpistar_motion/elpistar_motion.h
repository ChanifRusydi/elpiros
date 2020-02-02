#ifndef ELPISTAR_MOTION_CONTROLLERS_H
#define ELPISTAR_MOTION_CONTROLLERS_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
#include <elpistar_imu/EulerIMU.h>
#include <elpistar_msgs/DXLServer.h>
#include <bcm2835.h>

#define WALK_FREQUENCY (15)
#define WALK 0
#define FRONT_STANDUP 1
#define BACK_STANDUP 2
#define CONTROL 3
#define SPIN_R 4
#define SPIN_L 5
#define SHIFT_L 6
#define SHIFT_R 7
#define FRONT_STANDUP_OLD 8
#define SIT 9
#define SIT_OLD 10
#define BACK_STANDUP_OLD 11
#define START_BUTTON RPI_BPLUS_GPIO_J8_40
typedef struct PID{
   float Kp,Ki,Kd,Ts;
   float P,I,D;
   float error,last_error,SP;
   float u;
   bool en;
   bool walk_r;
};
bool debug_mode;
bool start=false;
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
  ros::ServiceClient camera_client_;
  std_srvs::Trigger camera_state_;

  //Elpistar Motion Controller Parameter
  int robot_y;
  std::string robot_name_;
  int prev_state;
  int fall_state;
  public:
   ElpistarMotionController();
   ~ElpistarMotionController();
   void robotControl();
   void walk(int step);
   void walk_ready();
   void stop();
   void front_standup();
   void back_standup();
   void back();
   void spin_r(int step);
   void spin_l(int step);
   void shift_l(int step);
   void shift_r(int step);
   
   void sit();
   void sit_old();
   void back_standup_old();
   void euler_pos_cb(const elpistar_imu::EulerIMU::ConstPtr &msg);   
  private:
   void initPublisher();
   void initSubscriber();
   void initClient();
   
   PID phi_ctrl;
   void motion(uint8_t type, uint8_t pn);
};

#endif
