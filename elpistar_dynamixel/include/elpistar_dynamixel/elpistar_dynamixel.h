#ifndef ELPISTAR_DYNAMIXEL_CONTROLLERS_H
#define ELPISTAR_DYNAMIXEL_CONTROLLERS_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <vector>
#include <string>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <sensor_msgs/JointState.h>
#include <elpistar_msgs/DXLServer.h>

namespace elpistar_dynamixel{
// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

//
#define ITERATION_FREQUENCY  (100)
#define JOINT_NUM   20
// #define DEBUG

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber goal_joint_states_sub_;
//   ros::Subscriber goal_gripper_states_sub_;

  // ROS Service Server
  ros::ServiceServer move_dxl_server_;
  // ROS Service Client

  // Dynamixel Workbench Parameters
  std::string robot_name_;
  float protocol_version_;

  DynamixelWorkbench *joint_controller_;
//   DynamixelWorkbench *gripper_controller_;

  std::map<std::string, uint32_t> dynamixel_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
//   std::vector<uint8_t> gripper_id_;

  std::string joint_mode_;
//   std::string gripper_mode_;

 public:
  DynamixelController();
  ~DynamixelController();
  bool control_loop();

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void initServer();
  bool getDynamixelInfo(const std::string yaml_file);
  void getDynamixelInst();
  void setOperatingMode();
  bool initDynamixels();
  void setSyncFunction();
  void readPosition(int *value);
  void readVelocity(double *value);
  void updateJointStates();

  bool move_dxl(elpistar_msgs::DXLServer::Request &req,
                elpistar_msgs::DXLServer::Response &res);
};
}

#endif
