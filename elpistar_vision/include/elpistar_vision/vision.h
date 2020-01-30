#ifndef ELPISTAR_VISION_H
#define ELPISTAR_VISION_H

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Trigger.h>

class ElpistarVisionController{
    private:
      ros::ServiceServer line_state_server_;
      ros::NodeHandle nh;
      image_transport::ImageTransport it;     
      image_transport::Publisher image_pub;
      int8_t state;
      std::string robot_name_;
      cv::VideoCapture cap;

    public:
      ElpistarVisionController();
      ~ElpistarVisionController();
      // bool update_vision(std_srvs::Trigger::Request &req,
      //                    std_srvs::Trigger::Response &res);
      void update_vision();
    private:
      bool line_state(std_srvs::Trigger::Request &req,
                      std_srvs::Trigger::Response &res); 
};
	
#endif