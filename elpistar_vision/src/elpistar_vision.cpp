// ConsoleApplication1.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
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
	cap.set(3, 320.0);
	cap.set(4, 240.0);

	while (ros::ok()) {
		Mat frame, gray, blur, th;
		cap.read(frame);
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		GaussianBlur(gray, blur, Size(5, 5), 0);
		threshold(blur, th, 35, 255, THRESH_BINARY + THRESH_OTSU);
		
		vector<vector<Point>> contours;
		vector<Vec4i> hierarchy;
		findContours(th, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		Scalar color = Scalar(0, 255, 0);
		drawContours(frame, contours, -1, color, 3);


		imshow("Image", frame);


		Moments M;
		int cx, cy;
		string status;
		for (int i = 0; i <= contours.size() - 1; i++) {
			double area = contourArea(contours[i]);
			if (area >= 500) {
				M = moments(contours[i]);
				cx = (int)(M.m10 / M.m00);
				cy = (int)(M.m01 / M.m00);

				if (cx <= 154) {
					status = "Kiri";
				}
				else if (cx >= 166) {
					status = "Kanan";
				}
				else {
					status = "Tengah";
				}

				cout << "Centroid X: " << cx;
				cout << " Centroid Y: " << cy;
				cout << " Status: " << status << endl;
			}
		}

		if (waitKey(30) == 27) {
			cout << "Key has been pressed" << endl;
			break;
		}

 ros_img=cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
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
