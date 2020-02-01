#include <elpistar_vision/vision.h>


void param_callback(elpistar_vision::VisionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d ", 
            config.image_thresh);
  vision_thresh=config.image_thresh;

}

ElpistarVisionController::ElpistarVisionController():nh(""),it(nh){
	robot_name_   = nh.param<std::string>("robot_name", "elpistar");
	line_state_server_  = nh.advertiseService(robot_name_+"/line", &ElpistarVisionController::line_state, this);
	image_pub = it.advertise(robot_name_+"/image_raw",1);
  vision_thresh=137;
	state=0;
	cap=cv::VideoCapture(0);	
	cap.set(3, 160.0);
	cap.set(4, 120.0);
	cap.set(cv::CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

}
ElpistarVisionController::~ElpistarVisionController(){
	ros::shutdown();	
}
bool ElpistarVisionController::line_state(std_srvs::Trigger::Request &req,
  										  std_srvs::Trigger::Response &res){
	std::stringstream ss;
	ss<<state;
	res.message=ss.str();
	res.success=true;
	return true;
}
void ElpistarVisionController::update_vision()
											 {
	std_msgs::Int8 line_state;
	sensor_msgs::ImagePtr ros_img;
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::Scalar color = cv::Scalar(0, 255, 0);
	
	cv::Mat frame, crop, gray, blur, th;	
	cv::Rect roi;
	cv::Moments M;
	
	int offset_x = 0;
	int offset_y = 60;
	int cx, cy;
	// string status;

	if(cap.read(frame)){
		roi.x = offset_x;
		roi.y = offset_y;
		roi.width = frame.size().width - (offset_x * 2);
		roi.height = frame.size().height - (offset_y * 2);
		crop=frame;
		//crop = frame(roi);

		cvtColor(crop, gray, cv::COLOR_BGR2GRAY);
		GaussianBlur(gray, blur, cv::Size(5, 5), 0);
		threshold(blur, th, vision_thresh, 255, cv::THRESH_BINARY);
		findContours(th, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
		cv::drawContours(crop, contours, -1, color, 3);

		// imshow("Image", frame);

		if (contours.size() > 0) {
			int c=0;
			for (int i = 1; i <= contours.size() - 1; i++) {
				if( contourArea(contours[c]) < contourArea(contours[i]) ){
					c=i;
				}
			}

			M = moments(contours[c]);
			if (M.m00 != 0) {
				cx = (int)(M.m10 / M.m00);
				cy = (int)(M.m01 / M.m00);
			} else {
				cx = (int)(M.m10 / 1);
				cy = (int)(M.m01 / 1);
			}
			cv::line(crop, cv::Point(cx, 0), cv::Point(cx, 240), cv::Scalar(255,0,0), 2);
			cv::line(crop, cv::Point(0, cy), cv::Point(320, cy), cv::Scalar(255,0,0), 2);
			cv::drawContours(crop, contours, -1,cv::Scalar(0,255,0));
			if (cx >= 120) {
				// res.message = "Kiri";
				ROS_INFO("Kiri");
				state=1;
			}

			else if (cx < 120  && cx > 50) {
				// res.message = "Tengah";
				ROS_INFO("Tengah");
				state=2;
			}

			else if(cx <= 50) {
				// res.message = "Kanan";
				ROS_INFO("Kanan");
				state=3;
			}
			
			
			// return true;
		}   
		else{
			// return false;	
		}
		ros_img=cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
		image_pub.publish(ros_img); 
	}
}
int main(int argc, char* argv[])
{
	ros::init(argc, argv,"elpistar_line_server");
	ElpistarVisionController vision_server;
	ros::Rate refresh_rate(30);
	dynamic_reconfigure::Server<elpistar_vision::VisionConfig> srv;
  	dynamic_reconfigure::Server<elpistar_vision::VisionConfig>::CallbackType f;
  	f = boost::bind(&param_callback, _1, _2);
  	srv.setCallback(f);

	while(ros::ok()){
		vision_server.update_vision();
		ros::spinOnce();
		refresh_rate.sleep();
	}
}
