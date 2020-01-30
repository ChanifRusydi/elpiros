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
  robot_y = priv_node_handle_.param<float>("robot_y",0);
  phi_ctrl.SP= priv_node_handle_.param<int32_t>("phi_SP",-1.5);
  phi_ctrl.Kp= priv_node_handle_.param<float>("phi_Kp",0);
  phi_ctrl.Ki= priv_node_handle_.param<float>("phi_Ki",0);
  phi_ctrl.Kd= priv_node_handle_.param<float>("phi_Kd",0);
  phi_ctrl.Ts= priv_node_handle_.param<float>("phi_Ts",0.2);
  phi_ctrl.en= false;
  printf("%.2f, %.2f, %.2f, %.2f",phi_ctrl.Kp, phi_ctrl.Ki, phi_ctrl.Kd, phi_ctrl.SP);
//  ros::shutdown();
  // initPublisher();
  initSubscriber();
  initClient();
  ROS_INFO("elpistar_motion_controller : Init OK!");
}

ElpistarMotionController::~ElpistarMotionController(){
  stop();
  ros::shutdown();
}

// void ElpistarMotionController::initPublisher(){
//   goal_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>(robot_name_+"/goal_joint_position",10);
// }
void ElpistarMotionController::initClient(){
  move_dxl_client_ = node_handle_.serviceClient<elpistar_msgs::DXLServer>(robot_name_ + "/move_dxl");
}
void ElpistarMotionController::initSubscriber(){
  position_sub_ = node_handle_.subscribe<elpistar_imu::EulerIMU>("/imu/euler",10, &ElpistarMotionController::euler_pos_cb, this);
}
void ElpistarMotionController::euler_pos_cb(const elpistar_imu::EulerIMU::ConstPtr &msg){
  sensor_msgs::JointState dxl;
  elpistar_msgs::DXLServer move_dxl;
  ros::Rate refresh(10);
  bool stat;
  uint16_t gp[20];
  if(phi_ctrl.walk_r){
    uint16_t goal[20]={235,788,279,744,462,561,358,666,507,516,346,677,240,783,647,376,507,516,372,512}; //walk_ready
    for(int i=0; i<20; i++) gp[i]=goal[i];
  }
  else{
    uint16_t goal[20]={175,728,279,744,462,561,358,666,507,516,292,674,248,775,614,352,507,516,372,512}; //walk
    for(int i=0; i<20; i++) gp[i]=goal[i];
  }
  if(phi_ctrl.en){
    float phi=msg->phi;
    phi_ctrl.error=phi_ctrl.SP-phi;
    phi_ctrl.P=phi_ctrl.Kp*phi_ctrl.error;
    phi_ctrl.I=phi_ctrl.Ki*(phi_ctrl.error+phi_ctrl.last_error)*phi_ctrl.Ts;
    phi_ctrl.D=phi_ctrl.Kd*(phi_ctrl.error-phi_ctrl.last_error)/phi_ctrl.Ts;
    phi_ctrl.u=phi_ctrl.P+phi_ctrl.I+phi_ctrl.D;
    phi_ctrl.last_error=phi_ctrl.error;
    uint16_t speed[20]={767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767,767};
    gp[10]=mapd(72 + phi_ctrl.u, 0, 255)+256;
    gp[11]=mapd(183 - phi_ctrl.u, 0, 255)+512;
    gp[12]=mapd(240 - phi_ctrl.u, 0, 255);
    gp[13]=mapd(15 + phi_ctrl.u, 0, 255)+768;
    gp[14]=mapd(135 - phi_ctrl.u/1.2, 0, 255)+512;
    gp[15]=mapd(120 + phi_ctrl.u/1.2, 0, 255)+256;
    speed[10]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    speed[11]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    speed[12]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    speed[13]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    speed[14]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    speed[15]=mapd(170+ abs(phi_ctrl.u*2), 5, 250);
    for(uint8_t i=0; i<20; i++){
      if(i==10 )
        dxl.position.push_back(gp[i]-robot_y);
      else if(i==11)
        dxl.position.push_back(gp[i]+robot_y);
      else
        dxl.position.push_back(gp[i]);
      dxl.velocity.push_back(speed[i]);
    }
    move_dxl.request.jointstate=dxl;
    stat=move_dxl_client_.call(move_dxl);
    /*while(!stat){
      refresh.sleep();
      stat=move_dxl_client_.call(move_dxl);
    }*/
    printf("Kontrol Aktif");
    phi_ctrl.en=false;
    phi_ctrl.walk_r=false;
  }
    printf("%7.2f\n",msg->phi);
}
void ElpistarMotionController::motion(uint8_t type, uint8_t pn){
  sensor_msgs::JointState dxl;
  elpistar_msgs::DXLServer move_dxl;
  ros::Rate refresh(10);
  bool stat;
  switch(type){
    case WALK:{
      switch(pn){
        case 0:
        {
          uint16_t gp[20]={169,722,279,744,462,561,358,666,513,522,297,665,246,769,621,348,513,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:
        {  
          uint16_t gp[20]={166,719,279,744,462,561,358,666,518,527,303,656,248,761,625,348,518,527,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:
        {
          uint16_t gp[20]={167,720,279,744,462,561,358,666,519,533,309,655,252,761,628,346,523,532,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:
        {
          uint16_t gp[20]={171,724,279,744,462,561,358,666,513,541,316,667,257,778,629,341,527,537,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:
        {
          uint16_t gp[20]={179,732,279,744,462,561,358,666,504,550,322,687,262,801,630,338,530,541,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:
        {
          uint16_t gp[20]={189,742,279,744,462,561,358,666,496,556,327,709,266,820,631,342,532,543,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:
        {
          uint16_t gp[20]={201,754,279,744,462,561,358,666,493,558,330,729,267,827,633,354,533,544,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:
        {
          uint16_t gp[20]={215,768,279,744,462,561,358,666,496,556,332,741,266,820,636,374,532,543,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:
        {
          uint16_t gp[20]={230,783,279,744,462,561,358,666,504,550,333,745,262,801,641,396,530,541,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:
        {
          uint16_t gp[20]={246,799,279,744,462,561,358,666,513,541,334,742,257,778,647,416,527,537,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:
        {
          uint16_t gp[20]={260,813,279,744,462,561,358,666,519,533,335,737,252,761,654,428,523,532,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:
        {
          uint16_t gp[20]={274,827,279,744,462,561,358,666,518,527,338,735,248,761,660,427,518,527,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:
        {
          uint16_t gp[20]={285,838,279,744,462,561,358,666,513,522,342,735,246,769,666,418,513,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:
        {
          uint16_t gp[20]={294,847,279,744,462,561,358,666,507,516,349,731,248,775,671,409,507,516,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:
        {
          uint16_t gp[20]={300,853,279,744,462,561,358,666,501,510,358,726,254,777,675,402,501,510,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:
        {
          uint16_t gp[20]={303,856,279,744,462,561,358,666,496,505,367,720,262,775,675,398,496,505,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:
        {
          uint16_t gp[20]={302,855,279,744,462,561,358,666,490,504,368,714,262,771,677,395,491,500,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:
        {
          uint16_t gp[20]={298,851,279,744,462,561,358,666,482,510,356,707,245,766,682,394,486,496,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:
        {
          uint16_t gp[20]={290,843,279,744,462,561,358,666,473,519,336,701,222,761,685,393,482,493,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:
        {
          uint16_t gp[20]={280,833,279,744,462,561,358,666,467,527,314,696,203,757,681,392,480,491,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:
        {
          uint16_t gp[20]={268,821,279,744,462,561,358,666,465,530,294,693,196,756,669,390,479,490,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:
        {
          uint16_t gp[20]={254,807,279,744,462,561,358,666,467,527,282,691,203,757,649,387,480,491,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:
        {
          uint16_t gp[20]={239,792,279,744,462,561,358,666,473,519,278,690,222,761,627,382,482,493,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:
        {
          uint16_t gp[20]={223,776,279,744,462,561,358,666,482,510,281,689,245,766,607,376,486,496,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:
        {
          uint16_t gp[20]={209,762,279,744,462,561,358,666,490,504,286,688,262,771,595,369,491,500,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:
        {
          uint16_t gp[20]={195,748,279,744,462,561,358,666,496,505,288,685,262,775,596,363,496,505,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:
        {
          uint16_t gp[20]={184,737,279,744,462,561,358,666,501,510,288,681,254,777,605,357,501,510,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:
        {
          uint16_t gp[20]={175,728,279,744,462,561,358,666,507,516,292,674,248,775,614,352,507,516,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          phi_ctrl.en=true;
          break;
        }
        case 100:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,346,677,240,783,647,376,507,516,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          phi_ctrl.walk_r=true;
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
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={361, 662, 496, 527, 501, 522, 353, 670, 508, 515, 346, 677, 282, 741, 617, 406, 508, 515, 512, 512};           
          uint16_t speed[20]={170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={611, 412, 580, 491, 107, 868, 353, 670, 508, 515, 346, 677, 282, 741, 617, 406, 508, 515, 512, 512};           
          uint16_t speed[20]={255,  255,  255,  255,  255,  255,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  170,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={611, 412, 580, 491, 107, 868, 353, 670, 498, 525, 55, 968, 121, 902, 777, 246, 507, 519, 512, 512};           
          uint16_t speed[20]={255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  255,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={407, 616, 264, 759, 454, 569, 354, 669, 512, 511, 80, 943, 38, 985, 705, 318, 512, 511, 512, 512};           
          uint16_t speed[20]={96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={378, 645, 264, 759, 454, 569, 354, 669, 512, 511, 80, 943, 86, 937, 609, 414, 512, 511, 512, 512};           
          uint16_t speed[20]={48,  48,  96,  96,  96,  96,  96,  96,  96,  96,  48,  48,  48,  48,  48,  48,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 277, 746, 240, 783, 624, 399, 512, 511, 512, 512};           
          uint16_t speed[20]={56,  56,  56,  56,  56,  56,  56,  56,  56,  56,  85,  85,  66,  66,  66,  66,  96,  96,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
            dxl.velocity.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 333, 690, 240, 783, 647, 376, 507, 516, 512, 512};           
          uint16_t speed[20]={48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  48,  512,  512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
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
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={16, 1007, 509, 514, 335, 688, 353, 666, 507, 516, 341, 682, 128, 895, 443, 580, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={16, 1007, 509, 514, 175, 848, 353, 666, 507, 516, 283, 740, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={16, 1007, 253, 770, 462, 561, 353, 666, 507, 516, 58, 965, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 965, 128, 895, 459, 564, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 885, 831, 895, 459, 564, 507, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={192, 831, 253, 770, 462, 561, 497, 602, 507, 516, 58, 885, 831, 895, 927, 468, 545, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={160, 831, 253, 770, 462, 561, 497, 602, 507, 516, 21, 805, 80, 895, 511, 460, 562, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={160, 831, 306, 770, 462, 561, 497, 602, 507, 516, 21, 768, 80, 920, 545, 323, 562, 580, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
          uint16_t gp[20]={160, 831, 306, 770, 462, 561, 497, 602, 507, 516, 112, 768, 102, 920, 545, 323, 511, 580, 512, 512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
          uint16_t gp[20]={442, 581, 264, 759, 454, 569, 354, 669, 512, 511, 378, 645, 102, 920, 545, 323, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={378, 645, 264, 759, 454, 569, 354, 669, 512, 511, 80, 943, 86, 937, 609, 414, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 277, 746, 240, 783, 619, 404, 512, 511, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={235, 788, 279, 744, 462, 561, 358, 666, 507, 516, 333, 690, 240, 783, 647, 376, 507, 516, 512, 512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
            dxl.position.push_back(gp[i]-robot_y);
          else if(i==11)
            dxl.position.push_back(gp[i]+robot_y);
          else
            dxl.position.push_back(gp[i]);
          }
          break;
        }
      }
      break;
    }

    case SPIN_R:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,513,522,333,694,241,781,647,381,513,522,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
            dxl.position.push_back(gp[i]-robot_y);
          else if(i==11)
            dxl.position.push_back(gp[i]+robot_y);
          else
            dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,519,527,336,693,245,899,647,385,519,527,372,512};      
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={235,788,279,744,462,561,417,606,521,534,340,696,250,777,646,386,524,533,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={235,788,279,744,462,561,413,610,515,543,345,703,257,790,643,380,529,538,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={235,788,279,744,462,561,406,617,506,551,348,712,263,807,641,372,577,542,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={235,788,279,744,462,561,398,625,498,558,350,718,267,821,638,365,534,546,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235,788,279,744,788,521,534,462,561,388,635,495,561,349,719,269,826,636,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235,788,279,744,462,561,379,644,498,559,347,714,267,821,636,361,535,546,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={235,788,279,744,462,561,370,653,506,552,344,706,263,807,637,366,533,543,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
          uint16_t gp[20]={235,788,279,744,462,561,363,660,516,544,340,696,257,790,639,373,529,539,372,512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
          uint16_t gp[20]={235,788,279,744,462,561,359,664,521,535,337,689,250,777,642,379,525,534,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,520,528,334,688,245,776,644,380,520,528,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,514,522,332,691,241,781,646,377,514,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={491,788,279,744,462,561,358,666,507,516,331,692,240,783,647,376,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,501,509,332,691,242,782,646,377,501,509,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,495,503,335,689,247,778,643,379,495,503,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:{
          uint16_t gp[20]={235,788,279,744,462,561,359,664,488,502,334,686,246,773,644,381,489,498,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:{
          uint16_t gp[20]={235,788,279,744,462,561,363,660,479,507,327,683,233,766,650,384,484,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:{
          uint16_t gp[20]={235,788,279,744,462,561,370,653,471,517,317,679,216,760,657,386,480,490,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:{
          uint16_t gp[20]={235,788,279,744,462,561,379,644,464,525,309,676,202,756,662,387,477,488,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:{
          uint16_t gp[20]={235,788,279,744,462,561,388,635,462,528,304,674,197,242,662,387,476,488,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:{
          uint16_t gp[20]={235,788,279,744,462,561,398,625,465,525,305,673,202,756,658,385,477,489,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:{
          uint16_t gp[20]={235,788,279,744,462,561,406,617,472,517,311,675,216,760,651,382,481,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:{
          uint16_t gp[20]={235,788,279,744,462,561,413,610,480,508,320,678,233,766,643,380,485,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:{
          uint16_t gp[20]={235,788,279,744,462,561,417,606,489,502,327,683,246,773,637,377,490,499,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,496,504,330,687,247,778,638,376,496,504,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,501,510,329,690,242,782,642,376,501,510,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,507,516,330,693,240,783,645,378,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
    

      }
      break;
    }

    case SPIN_L:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,514,522,322,701,241,781,646,377,514,522,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,520,528,324,698,245,776,644,380,520,528,372,512};      
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={235,788,279,744,462,561,359,664,521,535,327,699,250,777,642,379,525,534,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={235,788,279,744,462,561,363,660,516,544,330,706,257,790,639,373,529,539,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={235,788,279,744,462,561,370,653,506,552,334,716,263,807,637,366,533,543,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={235,788,279,744,462,561,379,644,498,754,337,724,267,821,636,361,535,546,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235,788,279,744,462,561,388,635,495,561,339,729,269,826,636,361,535,547,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235,788,279,744,462,561,398,625,498,558,340,728,267,821,638,365,534,546,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={235,788,279,744,462,561,406,617,506,551,338,722,263,807,641,372,532,542,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
          uint16_t gp[20]={235,788,279,744,462,561,413,610,515,543,335,713,257,790,643,380,529,538,372,512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
          uint16_t gp[20]={235,788,279,744,462,561,417,606,521,534,330,706,250,777,646,386,524,533,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,519,527,326,703,245,776,647,385,524,533,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,513,522,323,704,241,781,647,381,513,522,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,507,516,320,703,240,783,645,378,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,501,510,319,700,242,782,642,376,501,510,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:{
          uint16_t gp[20]={235,788,279,744,462,561,419,604,496,504,320,697,247,778,638,376,496,504,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:{
          uint16_t gp[20]={235,788,279,744,462,561,417,606,489,502,317,693,246,773,637,377,490,499,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:{
          uint16_t gp[20]={235,788,279,744,462,561,413,610,480,508,310,688,233,766,643,380,485,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:{
          uint16_t gp[20]={235,788,279,744,462,561,406,617,472,517,301,685,216,760,651,382,481,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:{
          uint16_t gp[20]={235,788,279,744,462,561,398,625,465,525,295,683,202,756,658,385,477,489,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:{
          uint16_t gp[20]={235,788,279,744,462,561,388,635,462,528,294,664,197,754,662,387,476,488,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:{
          uint16_t gp[20]={235,788,279,744,462,561,379,644,464,525,299,686,202,756,662,387,477,488,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:{
          uint16_t gp[20]={235,788,279,744,462,561,370,653,471,517,307,689,216,760,657,641,480,490,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:{
          uint16_t gp[20]={235,788,279,744,462,561,363,660,479,507,317,693,233,766,650,384,484,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:{
          uint16_t gp[20]={235,788,279,744,462,561,359,664,488,502,324,696,246,773,644,381,489,498,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,495,503,325,699,247,778,643,379,495,503,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,501,509,322,701,242,782,646,377,501,509,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,321,702,240,783,647,376,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
    

      }
      break;
    }


    case SHIFT_R:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,479,559,325,692,251,759,641,388,479,559,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,487,566,324,686,250,747,642,394,487,566,372,512};      
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,491,573,347,684,251,743,641,396,495,572,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,489,581,326,646,254,756,640,390,503,576,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,485,587,328,700,258,776,638,380,511,578,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,482,590,330,710,262,795,636,370,519,577,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235,788,279,235,788,279,744,462,485,587,331,715,264,806,635,365,525,573,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,493,579,332,715,265,806,634,365,529,566,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,506,567,330,711,262,798,636,369,532,557,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,518,553,328,705,258,785,638,375,532,548,372,512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,525,540,325,700,251,775,641,380,529,539,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,523,531,322,700,245,775,644,380,523,531,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,515,524,320,702,241,780,646,378,515,524,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,319,704,240,783,647,376,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,499,508,321,703,243,782,645,377,499,508,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,492,500,323,701,248,778,643,379,492,500,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,483,498,323,698,248,772,643,382,484,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,470,505,318,695,238,765,648,385,475,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,456,517,312,693,225,761,654,387,466,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,444,530,308,691,217,758,658,389,457,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,436,538,308,692,217,759,658,388,450,498,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,433,541,313,693,228,761,653,387,446,504,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,436,538,323,695,247,765,643,385,445,512,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,442,534,333,697,267,769,633,383,447,520,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,450,532,339,698,280,772,627,382,451,528,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,457,536,337,699,276,773,629,381,457,536,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,464,544,331,698,264,772,635,382,464,544,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,321,702,240,783,647,376,507,516,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
    

      }
      break;
    }


    case SHIFT_L:{
      switch(pn){
        case 0:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,515,524,320,702,241,780,646,378,515,524,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 1:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,523,531,322,700,245,775,644,380,523,531,372,512};      
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 2:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,525,540,325,700,251,775,641,380,529,539,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 3:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,518,553,328,705,258,785,638,375,532,548,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 4:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,506,567,330,711,262,798,636,369,532,557,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 5:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,493,579,332,715,265,806,634,365,529,566,372,512};        
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 6:{
          uint16_t gp[20]={235,788,279,235,788,279,744,462,485,587,331,715,264,806,635,365,525,573,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 7:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,482,590,330,710,262,795,636,370,519,577,372,512};          
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 8:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,485,587,328,700,258,776,638,380,511,578,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 9:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,489,581,326,690,254,756,640,390,503,576,372,512};
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 10:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,491,573,325,684,251,743,641,396,495,572,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 11:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,487,566,324,686,250,747,642,394,487,566,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 12:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,479,559,325,692,251,759,641,388,479,559,372,512};           
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 13:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,472,551,327,440,256,767,639,384,472,551,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 14:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,464,544,331,698,264,772,635,382,464,544,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 15:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,457,536,337,699,276,773,629,381,457,536,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 16:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,450,532,339,698,280,772,627,382,451,528,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 17:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,442,534,333,697,267,769,633,383,447,520,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 18:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,436,538,323,695,247,765,643,385,445,512,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 19:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,433,541,313,693,228,761,653,387,446,504,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 20:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,436,538,308,692,217,759,658,388,450,498,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 21:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,444,530,308,691,217,758,658,389,457,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 22:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,456,517,312,693,225,761,654,387,466,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 23:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,470,505,318,695,238,765,648,385,475,491,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 24:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,483,498,323,698,248,772,643,382,484,494,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 25:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,492,500,323,701,248,778,643,379,492,500,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 26:{
         uint16_t gp[20]={235,788,279,744,462,561,358,666,499,508,321,703,243,782,645,377,499,508,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
        case 27:{
          uint16_t gp[20]={235,788,279,744,462,561,358,666,507,516,319,704,240,783,647,376,507,512,372,512};         
          for(uint8_t i=0; i<20; i++){
            if(i==10 )
              dxl.position.push_back(gp[i]-robot_y);
            else if(i==11)
              dxl.position.push_back(gp[i]+robot_y);
            else
              dxl.position.push_back(gp[i]);
          }
          break;
        }
    

      }
      break;
    }

    

  }
  move_dxl.request.jointstate=dxl;
  stat=move_dxl_client_.call(move_dxl);
  /*while(!stat){
    refresh.sleep();
    stat=move_dxl_client_.call(move_dxl);
  }*/
}
  
void ElpistarMotionController::walk(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(28);
  ros::Rate step_time(2);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(WALK,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
    step_time.sleep();
  }
}

void ElpistarMotionController::shift_r(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(28);
  ros::Rate step_time(2);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(SHIFT_R,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
    step_time.sleep();
  }
}


void ElpistarMotionController::shift_l(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(28);
  ros::Rate step_time(2);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(SHIFT_L,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
    step_time.sleep();
  }
}
void ElpistarMotionController::spin_r(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(28);
  ros::Rate step_time(2);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(SPIN_R,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
    step_time.sleep();
  }
}

void ElpistarMotionController::spin_l(int step){
  uint8_t phase=28;
  ros::Rate loop_rate(28);
  ros::Rate step_time(2);
  for(int count=0; count<step; count++){
    for(int i=0; i<phase; i++){
      motion(SPIN_L,i);
      ros::spinOnce();
      loop_rate.sleep();
    }
    step_time.sleep();
  }
}

void ElpistarMotionController::walk_ready(){
    motion(WALK,100);
    ros::spinOnce();
}
void ElpistarMotionController::front_standup(){
  uint8_t phase=4;
  ros::Rate loop_rate(5);
  ros::Rate delay(0.1);
  for(int i=0; i<phase; i++){
    motion(FRONT_STANDUP,i);
    ros::spinOnce();
    loop_rate.sleep();
    if(i==3)
     delay.sleep();
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
  ros::Rate transition(0.2);

//  ros::Rate loop(5);
  //ros::spin();
  // ros::shutdown();
//   while (ros::ok())
//   {
    motion_controller.walk_ready();
    transition.sleep();
//    motion_controller.walk(30);
//    motion_controller.spin_r(3);  
//    motion_controller.front_standup();
    motion_controller.shift_r(3);
//    transition.sleep();
//    motion_controller.shift_l(10);
//    ros::spinOnce();
//    loop.sleep();
//   }

  return 0;
}
