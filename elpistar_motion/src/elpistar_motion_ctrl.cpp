#include <elpistar_motion/elpistar_motion.h>
int mapd(int x, int min, int max)
{
  if(x<=max){
    if(x>=min)
      return x;
    else{
      return min;  
    }
  }
  else{
    return max;
  }
}
ElpistarMotionController::ElpistarMotionController() :node_handle_(""),
     priv_node_handle_("~")
{
  robot_name_   = node_handle_.param<std::string>("robot_name", "elpistar");
  phi_ctrl.SP= priv_node_handle_.param<float>("phi_SP",-1.5);
  phi_ctrl.Kp= priv_node_handle_.param<float>("phi_Kp",0);
  phi_ctrl.Ki= priv_node_handle_.param<float>("phi_Ki",0);
  phi_ctrl.Kd= priv_node_handle_.param<float>("phi_Kd",0);
  phi_ctrl.Ts= priv_node_handle_.param<float>("phi_Ts",0.2);
  phi_ctrl.en= false;
  printf("%.2f, %.2f, %.2f, %.2f",phi_ctrl.Kp, phi_ctrl.Ki, phi_ctrl.Kd, phi_ctrl.SP);
//  ros::shutdown();
  initPublisher();
  initSubscriber();
  ROS_INFO("elpistar_motion_controller : Init OK!");
}

ElpistarMotionController::~ElpistarMotionController(){
  stop();
  ros::shutdown();
}

void ElpistarMotionController::initPublisher(){
  goal_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_+"/goal_joint_position",10);
}
void ElpistarMotionController::initSubscriber(){
  position_sub_ = node_handle_.subscribe<elpistar_imu::EulerIMU>("/imu/euler",10, &ElpistarMotionController::euler_pos_cb, this);
}
void ElpistarMotionController::euler_pos_cb(const elpistar_imu::EulerIMU::ConstPtr &msg){
  sensor_msgs::JointState dxl;
  // uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,346,677,240,783,647,376,507,516,372,512}; walk_ready
  uint16_t gp[20]={175,728,279,744,462,561,358,666,507,516,292,674,248,775,614,352,507,516,372,512}; //walk
  if(phi_ctrl.en){
    float phi=msg->phi;
    phi_ctrl.error=phi_ctrl.SP-phi;
    phi_ctrl.P=phi_ctrl.Kp*phi_ctrl.error;
    phi_ctrl.I=phi_ctrl.Ki*(phi_ctrl.error+phi_ctrl.last_error)*phi_ctrl.Ts;
    phi_ctrl.D=phi_ctrl.Kd*(phi_ctrl.error-phi_ctrl.last_error)/phi_ctrl.Ts;
    phi_ctrl.u=phi_ctrl.P+phi_ctrl.I+phi_ctrl.D;
    phi_ctrl.last_error=phi_ctrl.error;
    gp[10]=mapd(72 + phi_ctrl.u, 0, 255)+256;
    gp[11]=mapd(183 - phi_ctrl.u, 0, 255)+512;
    gp[12]=mapd(240 - phi_ctrl.u, 0, 255);
    gp[13]=mapd(15 + phi_ctrl.u, 0, 255)+768;
    gp[14]=mapd(135 - phi_ctrl.u, 0, 255)+512;
    gp[15]=mapd(120 + phi_ctrl.u, 0, 255)+256;
    for(uint8_t i=0; i<20; i++){
      dxl.position.push_back(gp[i]);
    }
    goal_joint_states_pub_.publish(dxl);
    printf("%7.2f\n",msg->phi);
  }
}
void ElpistarMotionController::motion(uint8_t type, uint8_t pn){
  sensor_msgs::JointState dxl;
  
  switch(type){
    case WALK:
    {
      switch(pn){
        case 0:
        {
          uint16_t gp[20]={169,722,279,744,462,561,358,666,513,522,297,665,246,769,621,348,513,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:
        {  
          uint16_t gp[20]={166,719,279,744,462,561,358,666,518,527,303,656,248,761,625,348,518,527,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:
        {
          uint16_t gp[20]={167,720,279,744,462,561,358,666,519,533,309,655,252,761,628,346,523,532,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:
        {
          uint16_t gp[20]={171,724,279,744,462,561,358,666,513,541,316,667,257,778,629,341,527,537,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:
        {
          uint16_t gp[20]={179,732,279,744,462,561,358,666,504,550,322,687,262,801,630,338,530,541,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:
        {
          uint16_t gp[20]={189,742,279,744,462,561,358,666,496,556,327,709,266,820,631,342,532,543,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:
        {
          uint16_t gp[20]={201,754,279,744,462,561,358,666,493,558,330,729,267,827,633,354,533,544,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:
        {
          uint16_t gp[20]={215,768,279,744,462,561,358,666,496,556,332,741,266,820,636,374,532,543,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:
        {
          uint16_t gp[20]={230,783,279,744,462,561,358,666,504,550,333,745,262,801,641,396,530,541,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:
        {
          uint16_t gp[20]={246,799,279,744,462,561,358,666,513,541,334,742,257,778,647,416,527,537,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:
        {
          uint16_t gp[20]={260,813,279,744,462,561,358,666,519,533,335,737,252,761,654,428,523,532,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:
        {
          uint16_t gp[20]={274,827,279,744,462,561,358,666,518,527,338,735,248,761,660,427,518,527,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:
        {
          uint16_t gp[20]={285,838,279,744,462,561,358,666,513,522,342,735,246,769,666,418,513,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:
        {
          uint16_t gp[20]={294,847,279,744,462,561,358,666,507,516,349,731,248,775,671,409,507,516,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:
        {
          uint16_t gp[20]={300,853,279,744,462,561,358,666,501,510,358,726,254,777,675,402,501,510,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:
        {
          uint16_t gp[20]={303,856,279,744,462,561,358,666,496,505,367,720,262,775,675,398,496,505,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:
        {
          uint16_t gp[20]={302,855,279,744,462,561,358,666,490,504,368,714,262,771,677,395,491,500,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:
        {
          uint16_t gp[20]={298,851,279,744,462,561,358,666,482,510,356,707,245,766,682,394,486,496,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:
        {
          uint16_t gp[20]={290,843,279,744,462,561,358,666,473,519,336,701,222,761,685,393,482,493,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:
        {
          uint16_t gp[20]={280,833,279,744,462,561,358,666,467,527,314,696,203,757,681,392,480,491,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:
        {
          uint16_t gp[20]={268,821,279,744,462,561,358,666,465,530,294,693,196,756,669,390,479,490,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:
        {
          uint16_t gp[20]={254,807,279,744,462,561,358,666,467,527,282,691,203,757,649,387,480,491,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:
        {
          uint16_t gp[20]={239,792,279,744,462,561,358,666,473,519,278,690,222,761,627,382,482,493,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:
        {
          uint16_t gp[20]={223,776,279,744,462,561,358,666,482,510,281,689,245,766,607,376,486,496,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:
        {
          uint16_t gp[20]={209,762,279,744,462,561,358,666,490,504,286,688,262,771,595,369,491,500,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:
        {
          uint16_t gp[20]={195,748,279,744,462,561,358,666,496,505,288,685,262,775,596,363,496,505,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:
        {
          uint16_t gp[20]={184,737,279,744,462,561,358,666,501,510,288,681,254,777,605,357,501,510,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:
        {
          uint16_t gp[20]={175,728,279,744,462,561,358,666,507,516,292,674,248,775,614,352,507,516,372,512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          phi_ctrl.en=true;
          break;
        }
      }
      break;
    }
    case FRONT_STANDUP:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={234, 789, 509, 514, 462, 561, 353, 670, 508, 515, 346, 677, 282, 741, 617, 406, 508, 515, 512, 512};   
          uint16_t speed[20]={170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170};        
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={361, 662, 496, 527, 501, 522, 353, 670, 508, 515, 346, 677, 282, 741, 617, 406, 508, 515, 512, 512};           
          uint16_t speed[20]={170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={611, 412, 580, 491, 107, 868, 353, 670, 508, 515, 346, 677, 282, 741, 617, 406, 508, 515, 512, 512};           
          uint16_t speed[20]={255,  255,  255,  255,  255,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={611, 412, 580, 491, 107, 868, 353, 670, 498, 525, 55, 968, 121, 902, 777, 246, 507, 519, 512, 512};           
          uint16_t speed[20]={255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={407, 616, 264, 759, 454, 569, 354, 658, 512, 511, 80, 943, 38, 985, 705, 318, 512, 511, 512, 512};           
          uint16_t speed[20]={96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={378, 645, 264, 759, 454, 569, 354, 658, 512, 511, 80, 943, 86, 937, 609, 414, 512, 511, 512, 512};           
          uint16_t speed[20]={48,  48,  96,  96,  96,  96,  96,  96,  96,  96,  48,  48,  48,  48,  48,  48,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 277, 746, 240, 783, 519, 399, 512, 511, 512, 512};           
          uint16_t speed[20]={56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  85,  85,  66,  66,  66,  66,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 333, 690, 240, 783, 647, 376, 507, 516, 512, 512};           
          uint16_t speed[20]={48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  512,  512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
      }
      break;
    }
    case BACK_STANDUP:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={0, 1023, 509, 514, 462, 561, 353, 666, 507, 516, 341, 682, 128, 895, 443, 580, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={16, 1007, 509, 514, 335, 688, 353, 666, 507, 516, 341, 682, 128, 895, 443, 580, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={16, 1007, 509, 514, 175, 848, 353, 666, 507, 516, 283, 740, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={16, 1007, 253, 770, 462, 561, 353, 666, 507, 516, 58, 965, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 965, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 885, 831, 895, 459, 564, 507, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 885, 831, 895, 927, 468, 545, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={160, 831, 253, 770, 462, 561, 497, 602, 507, 516, 21, 805, 80, 895, 511, 460, 562, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={160, 831, 306, 770, 462, 561, 497, 602, 507, 516, 21, 768, 80, 920, 545, 323, 562, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
          uint16_t gp[20]={160, 831, 306, 770, 462, 561, 497, 602, 507, 516, 112, 768, 102, 920, 545, 323, 511, 580, 512, 512};
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
          uint16_t gp[20]={442, 581, 264, 759, 454, 569, 354, 669, 512, 511, 378, 645, 102, 920, 545, 323, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={378, 645, 264, 759, 454, 569, 354, 669, 512, 511, 80, 943, 86, 937, 609, 414, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 277, 746, 240, 783, 619, 404, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 333, 690, 240, 783, 647, 376, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            dxl.position.push_back(gp[i]);
          }
          break;
        }
      }
      break;
    }
  }
  goal_joint_states_pub_.publish(dxl);
}
  
void ElpistarMotionController::walk(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(WALK_FREQUENCY);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(WALK,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
}
void ElpistarMotionController::front_standup(){
  uint8_t phase=4;
  ros::Rate loop_rate(2);
  for(int i=0; i<phase; i++){
    motion(FRONT_STANDUP,i);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void ElpistarMotionController::back_standup(){
  uint8_t phase=14;
  ros::Rate loop_rate(1);
  for(int i=0; i<phase; i++){
    motion(BACK_STANDUP,i);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
void ElpistarMotionController::stop(){}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "elpistar_dynamixel_controller");
  ElpistarMotionController motion_controller;
  ros::Rate loop(10);
  //ros::spin();
  // ros::shutdown();
   while (ros::ok())
   {
//    motion_controller.walk(1);
//    motion_controller.front_standup();
    ros::spinOnce();
    loop.sleep();
   }

  return 0;
}
