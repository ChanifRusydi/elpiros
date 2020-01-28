#include <elpistar_dynamixel/elpistar_dynamixel.h>

using namespace elpistar_dynamixel;
double mapd(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

DynamixelController::DynamixelController()
    :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_   = priv_node_handle_.param<std::string>("robot_name", "elpistar");

  std::string device_name   = priv_node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = priv_node_handle_.param<int>("baud_rate", 1000000);
  protocol_version_         = priv_node_handle_.param<float>("protocol_version", 1.0);
  std::string dynamixel_info = priv_node_handle_.param<std::string>("dynamixel_info", "../config/elpistar_joint.yaml");

  joint_mode_   = priv_node_handle_.param<std::string>("joint_controller", "position_mode");

  joint_controller_   = new DynamixelWorkbench;

  joint_controller_->begin(device_name.c_str(), dxl_baud_rate);

  if(getDynamixelInfo(dynamixel_info)){
    ROS_INFO("Dynamixel Successfully Loaded");
  }
  else{
    ROS_ERROR("Dynamixel Load Failed");
    ros::shutdown();
    return;
  }

  getDynamixelInst();
  if(initDynamixels()){
    ROS_INFO("Dynamixel Init Success");
  }
    else{
    ROS_ERROR("Dynamixel Init Failed");
    ros::shutdown();
    return;
  }

  //initPublisher();
  initSubscriber();
  initServer();
  ROS_INFO("elpistar_dynamixel_controller : Init OK!");
}

DynamixelController::~DynamixelController()
{
  for(auto const& dxl:dynamixel_)
    joint_controller_->torqueOff((uint8_t)dxl.second);

  ros::shutdown();
}

void DynamixelController::initPublisher()
{
  joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_ + "/joint_states", 10);
}

void DynamixelController::initSubscriber()
{
  // goal_joint_states_sub_    = node_handle_.subscribe(robot_name_ + "/goal_joint_position", 10, &DynamixelController::goalJointPositionCallback, this);
}
void DynamixelController::initServer()
{
  move_dxl_server_=node_handle_.advertiseService(robot_name_ + "/move_dxl", &DynamixelController::move_dxl , this);
}

bool DynamixelController::getDynamixelInfo(const std::string yaml_file){
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);

      dynamixel_info_.push_back(info);
    }
  }

  return true;
}
void DynamixelController::getDynamixelInst()
{
  for(auto const& dxl:dynamixel_){
    bool result=true;
    uint16_t model_number = 0;
    const char* log = NULL;
    result = joint_controller_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      ros::shutdown();
      return;
    }
    else
    {      
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }
  ROS_INFO("Detected %d dynamixels",dynamixel_.size());
  setSyncFunction();
}

void DynamixelController::setSyncFunction()
{
  joint_controller_->addSyncWriteHandler(1, "Goal_Position");
  joint_controller_->addSyncWriteHandler(1, "Moving_Speed");
  

  if (protocol_version_ == 2.0)
  {
    joint_controller_->addSyncReadHandler(1, "Present_Position");
    joint_controller_->addSyncReadHandler(1, "Present_Velocity");
  }
}


bool DynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    joint_controller_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = joint_controller_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            if(log=="[RxPacketError] Out of range error!"){
	      ROS_ERROR("ABCD");
	    }
            else
              ROS_ERROR("%s", log);
	    
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }

    joint_controller_->torqueOn((uint8_t)dxl.second);
  }

  return true;
}

void DynamixelController::readPosition(int *value)
{
  int32_t get_joint_present_position[JOINT_NUM];
  int32_t *get_position_ptr = NULL;

  // if (protocol_version_ == 2.0)
  // {
  //   get_position_ptr = joint_controller_->syncRead("Present_Position");

  //   for (auto const dxl:dynamixel_)
  //     get_joint_present_position[dxl.second-1] = get_position_ptr[dxl.second-1];
  // }
  // else if (protocol_version_ == 1.0)
  // {
    for (auto const dxl:dynamixel_){
      joint_controller_->itemRead((uint8_t)dxl.second, "Present_Position", &get_joint_present_position[dxl.second-1]);
      value[dxl.second-1]=get_joint_present_position[dxl.second-1];
    }
  // }
  // for (auto const dxl:dynamixel_){
  //   value[index] = joint_controller_->convertValue2Radian(joint_id_.at(index), get_joint_present_position[index]);
  // }
}

void DynamixelController::readVelocity(double *value)
{
  int32_t get_joint_present_velocity[JOINT_NUM];
  int32_t *get_velocity_ptr = NULL;

  // if (protocol_version_ == 2.0)
  // {
  //   get_velocity_ptr = joint_controller_->syncRead("Present_Velocity");

  //   for (auto const dxl:dynamixel_)
  //     get_joint_present_velocity[dxl.second-1] = get_velocity_ptr[dxl.second-1];
  // }
  // else if (protocol_version_ == 1.0)
  // {
    for (auto const dxl:dynamixel_)
      joint_controller_->itemRead((uint8_t)dxl.second, "Present_Velocity", &get_joint_present_velocity[dxl.second-1]);
  // }

  for (auto const dxl:dynamixel_)
    value[dxl.second-1] = joint_controller_->convertValue2Velocity((uint8_t)dxl.second, get_joint_present_velocity[dxl.second-1]);
}

void DynamixelController::updateJointStates()
{
  sensor_msgs::JointState joint_state;

  float joint_states_eff[JOINT_NUM] = {0.0, };

  int get_joint_position[JOINT_NUM] = {0, };
  double get_joint_velocity[JOINT_NUM] = {0.0, };

  readPosition(get_joint_position);
  readVelocity(get_joint_velocity);
  
  joint_state.header.frame_id = "world";
  joint_state.header.stamp    = ros::Time::now();

  for (auto const dxl:dynamixel_){
    joint_state.name.push_back(dxl.first.c_str());
    joint_state.position.push_back(get_joint_position[dxl.second-1]);
    joint_state.velocity.push_back(get_joint_velocity[dxl.second-1]);
    joint_state.effort.push_back(joint_states_eff[dxl.second-1]);
  }

  joint_states_pub_.publish(joint_state);
}

bool DynamixelController::move_dxl(elpistar_msgs::DXLServer::Request &req,
                                   elpistar_msgs::DXLServer::Response &res)
{
  int32_t goal_joint_position[JOINT_NUM] = {0, };
  
  uint8_t ID[JOINT_NUM];
  bool is_moving=false;
  int32_t speed[JOINT_NUM]={767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767};
  bool stat;
  const char* log = NULL;
  for (auto const dxl:dynamixel_){
    int32_t data;
    goal_joint_position[dxl.second-1] = req.jointstate.position.at(dxl.second-1);
    ID[dxl.second-1] = dxl.second;
    //joint_controller_->itemRead(dxl.second, "Moving", &data);
    if(data==1){
      is_moving=true;
    }
  }
  if(req.jointstate.velocity.size()==20){
    for(auto const dxl:dynamixel_){
      speed[dxl.second-1]=req.jointstate.velocity.at(dxl.second-1);
    }
  }
  int32_t goal_position[JOINT_NUM] = {0, };

  if(!is_moving){
    stat= joint_controller_->syncWrite(1, ID, (uint8_t) 20, speed,(uint8_t) 1, &log);
    if(stat){
//      ROS_INFO("Sync Write Moving Speed Success");
    }
    else{
      ROS_ERROR("Sync Write Moving Speed : %s",log);
      ros::shutdown();
      return false;
    }

    stat= joint_controller_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, ID, (uint8_t) 20, goal_joint_position,(uint8_t) 1, &log);
    if(stat){
//      ROS_INFO("Sync Write Goal Position Success");
    }
    else{
      ROS_ERROR("Sync Write Goal Position Error : %s",log);
      ros::shutdown();
      return false;
    }
    res.status=true;
    return true;
  }
  else{
  //  ROS_INFO("Dynamixel not finished moving");
    res.status=false; 
    return false;
  }
}



bool DynamixelController::control_loop()
{
  // Read & Publish Dynamixel position
  updateJointStates();
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "elpistar_dynamixel_controller");
  DynamixelController dynamixel_controller;
  ros::Rate loop_rate(ITERATION_FREQUENCY);
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
  ROS_INFO("Dynamixel Server is Ready");
  ros::spin();
  // while (ros::ok())
  // {
  // //  dynamixel_controller.control_loop();
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }

//  ros::waitForShutdown();

  return 0;
}
