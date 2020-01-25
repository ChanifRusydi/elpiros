// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
 
using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{


	ros::init(argc, argv,"line_node");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Publisher image_pub = it.advertise("test_cam/image_raw",1);
	sensor_msgs::ImagePtr ros_img;
	VideoCapture cap(0);
	cap.set(3, 160.0);
	cap.set(4, 120.0);
	cap.set(CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));

	while (ros::ok()) {
		Mat frame, crop, gray, blur, th;
		Rect roi;
		cap.read(frame);

		int offset_x = 0;
		int offset_y = 60;
		roi.x = offset_x;
		roi.y = offset_y;
		roi.width = frame.size().width - (offset_x * 2);
		roi.height = frame.size().height - (offset_y * 2);

		crop = frame(roi);

		cvtColor(crop, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, blur, Size(5, 5), 0);
		threshold(blur, th, 60, 255, THRESH_BINARY);
		
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(th, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

		Scalar color = Scalar(0, 255, 0);
		drawContours(crop, contours, -1, color, 3);


	//	imshow("Image", frame);


		Moments M;
		int cx, cy;
		string status;


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
			line(crop, Point(cx, 0), Point(cx, 240), Scalar(255,0,0), 2);
			line(crop, Point(0, cy), Point(320, cy), Scalar(255,0,0), 2);
			drawContours(crop, contours, -1,Scalar(0,255,0));
			if (cx >= 120) {
				status = "Kiri";
			}
			if (cx < 120  && cx > 50) {
				status = "Tengah";
			}
			if(cx <= 50) {
				status = "Kanan";
			}
						
			cout << "Centroid X: " << cx;
			cout << " Centroid Y: " << cy;
			cout << " Status: " << status << endl;
		}      
	
		if (waitKey(30) == 27) {
			cout << "Key has been pressed" << endl;
			break;
		}

		ros_img=cv_bridge::CvImage(std_msgs::Header(), "bgr8", crop).toImageMsg();
		image_pub.publish(ros_img);
		ros::spinOnce();
	}
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
