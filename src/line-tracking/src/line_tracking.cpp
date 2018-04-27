#include <unistd.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <ctime>
#include <signal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;

//Setting up the ROS messanger tags
string point_cloud_frame_id = "lineTracking";
ros::Time point_cloud_time;

//The end task flag (may not be used)
static volatile int keepRunning = 1;

//Scaling Values for the pixels to meters
static volatile double xscale = 0.003787727;
static volatile double yscale = 0.00419224;
static volatile double yOffset = 0.17;
static volatile double xOffset = 0.7366;

//Initialize the Mats needed for calculations
static Mat frame;
static Mat HSV;
static Mat threshld;
static Mat Gaussian;
static Mat Can;
static Mat ctv;
static Mat Hough;
static Mat output;

//Vector that holds the detected lines
static vector<Vec4i> lines;

//The HSV ranges
static int H_MIN, H_MAX, S_MIN, S_MAX, V_MIN, V_MAX, S_TOP, S_BOT, X_SCALE, Y_SCALE;

static double X_SCALE2, Y_SCALE2;

static int horizScaleBig, vertScaleBig, vertCorrBig;

static volatile float horizOffset = 0.17, vertOffset = 0.7366;
static double horizScale, vertScale, vertCorr;
static volatile int horizPixels, vertPixels;

//cnt+c handeler (may not be needed with ros.spin())
void intHandler(int dummy) {
	keepRunning = 0;
}

//ROS Comms initialization
class ImageConverter {
	//Initialize node handeler
	ros::NodeHandle nh_;
	
	//Set up subscriber
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	cv_bridge::CvImagePtr cv_ptr;
	
	//Publisher initialization
	ros::Publisher pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("lines", 1);

public:
	//Subscribe to the ROS topic
	ImageConverter():it_(nh_) {
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 1,
		&ImageConverter::imageCb, this);
	}

	//Main image transform function
	void imageCb(const sensor_msgs::ImageConstPtr& msg) {
		
		//*Initialization Phase*//
		
		//Try to convert
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		//Catch errors and print to screen
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//Get the file with HSV values and inport them
		std::fstream HSVfile("LineTrackingFiles/HSV.txt", std::ios_base::in);
		HSVfile >> H_MIN >> H_MAX >> S_MIN >> S_MAX >> V_MIN >> V_MAX >> S_TOP >> S_BOT >> horizScale >> vertScale >> vertCorr;

		//Get the current frame
		frame = cv_ptr->image;

		//*Perspective shift phase*//
		
		//Initialize perspective shift quadralaterals
		Point2f inputQuad[4];
		Point2f outputQuad[4];

		// Lambda Matrix
		Mat lambda(2, 4, CV_32FC1);

		// Set the lambda matrix the same type and size as input
		lambda = Mat::zeros(frame.rows, frame.cols, frame.type());
		
		horizPixels = frame.cols;
		vertPixels = frame.rows;

		// The 4 points that select quadilateral on the input , from top-left in clockwise order
		// These four pts are the sides of the rect box used as input
		//inputQuad[0] = Point2f(100, 360);
		//inputQuad[1] = Point2f(1180, 360);
		//inputQuad[2] = Point2f(1940, 720);
		//inputQuad[3] = Point2f(-660, 720);
		inputQuad[0] = Point2f(0, frame.rows/2);
		inputQuad[1] = Point2f(frame.cols, frame.rows/2);
		inputQuad[2] = Point2f(frame.cols, frame.rows);
		inputQuad[3] = Point2f(0, frame.rows);
		// The 4 points where the mapping is to be done , from top-left in clockwise order
		outputQuad[0] = Point2f(-S_TOP, 0);
		outputQuad[1] = Point2f(frame.cols+S_TOP, 0);
		outputQuad[2] = Point2f(frame.cols-S_BOT, frame.rows);
		outputQuad[3] = Point2f(+S_BOT, frame.rows);

		// Get the Perspective Transform Matrix i.e. lambda
		lambda = getPerspectiveTransform(inputQuad, outputQuad);
		// Apply the Perspective Transform just found to the src image
		warpPerspective(frame, output, lambda, output.size());

		//*Discovery Phase*//
		
		//Color filtering
		//Convert the input to an HSV image
		cvtColor(output, HSV, COLOR_BGR2HSV);
		//Filter the image color by the input numbers from earlier
		inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), threshld);
		inRange(HSV, Scalar(0, 0, 0), Scalar(0, 0, 0), Hough);
		//Call the fuction to remove erronious color patches
		morphOps(threshld);

		//Detect Edges
		Canny(threshld, Can, 50, 200, 3);

		//Convert to black and white
		cvtColor(Can, ctv, CV_GRAY2BGR);

		//Detect lines
		HoughLinesP(Can, lines, 1, CV_PI/180, 50, 20, 10);

		//*Data conversion phase*//
		//Initialize PointCloud
		pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
		point_cloud.width = frame.cols;
		point_cloud.height = frame.rows;
		int size = frame.cols*frame.rows;
		point_cloud.points.resize(size);
		point_cloud.header.frame_id = "lineTracking";

		int index = 0;
		//For all points
		for (size_t i = 0; i < lines.size(); i++) {
			//Convert take one point out of the array
			Vec4i l = lines[i];
			line(ctv, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);

			//Calculate distances
			/*
			double p0 = ((yscale) * ((l[0]) - (frame.cols/2))) + yOffset;
			double p1 = (xscale * (frame.rows - (l[1]))) + xOffset;
			double p2 = ((yscale) * ((l[2]) - (frame.cols/2))) + yOffset;
			double p3 = (xscale * (frame.rows - (l[3]))) + xOffset;
			*/
			
			float p0 = horizPixToDist(l[0]);
			float p1 = vertPixToDist(l[1]);
			float p2 = horizPixToDist(l[2]);
			float p3 = vertPixToDist(l[3]);

			//Add points to the point cloud
			point_cloud.points[index].y = p1;
			point_cloud.points[index].z = 0.5;
			point_cloud.points[index].x = p0;
			index++;
			point_cloud.points[index].y = p3;
			point_cloud.points[index].z = 0.5;
			point_cloud.points[index].x = p2;

			//Print the points to the screen
			cout << p1 << " " << p0 << endl;
			cout << p3 << " " << p2 << endl;
			
			double tempX = p2, tempY = p3;
			while (dist(p0,p1,tempX,tempY)>0.001) {
				tempX = (p0+tempX)/2;
				tempY = (p1+tempY)/2;
				point_cloud.points[index].y = tempY;
				point_cloud.points[index].z = 0.5;
				point_cloud.points[index].x = tempX;
				index++;
			}

			tempX = p0;
			tempY = p1;
			while (dist(p2,p3,tempX,tempY)>0.001) {
				tempX = (p2+tempX)/2;
				tempY = (p3+tempY)/2;
				point_cloud.points[index].y = tempY;
				point_cloud.points[index].z = 0.5;
				point_cloud.points[index].x = tempX;
				index++;
			}
		}
		//Space things out
		cout << endl;
		
		//Show the monitoring window
		namedWindow( "Line Tracking", WINDOW_NORMAL );// Create a window for display.
		imshow("Line Tracking", ctv);
		waitKey(25);
		namedWindow( "Shift", WINDOW_NORMAL );// Create a window for display.
		imshow("Shift", output);
		waitKey(25);

		//*ROS Send Phase*//
		//Define the message
		sensor_msgs::PointCloud2 output;
		//Convert the point cloud to a ROS point cloud
		pcl::toROSMsg(point_cloud, output);
		//Fill out the details of the ROS message
		output.header.frame_id = point_cloud_frame_id; 
		output.header.stamp = point_cloud_time.now();
		output.height = 1;
		output.width = index;
		output.is_bigendian = false;
		output.is_dense = false;
		//Publish the point cloud
		pub_cloud.publish(output);

		sleep(0.1);
	}

	void morphOps(Mat &thresh) {
		//create structuring element that will be used to "dilate" and "erode" image.
		//the element chosen here is a 3px by 3px rectangle

		Mat erodeElement = getStructuringElement(MORPH_RECT, Size(2, 2));
		//dilate with larger element so make sure object is nicely visible
		Mat dilateElement = getStructuringElement(MORPH_RECT, Size(3, 3));

		//Erode the image
		//erode(thresh, thresh, erodeElement);
		//erode(thresh, thresh, erodeElement);

		//Dilate the image
		dilate(thresh, thresh, dilateElement);
		dilate(thresh, thresh, dilateElement);
	}

	cv_bridge::CvImagePtr getImage() {
		return cv_ptr;
	}
	
	float horizPixToDist(int pixel) {
		float dist = (horizScale*(pixel-(horizPixels/2)))+horizOffset;
		return dist;
	}

	int horizDistToPix(float dist) {
		float pixel = ((dist-horizOffset)/horizScale)+(horizPixels/2);
		return pixel;
	}

	float vertPixToDist(int pixel) {
		float dist = (vertScale*(vertPixels-pixel))+vertOffset;
		return dist;
	}

	int vertDistToPix(float dist) {
		float pixel = -((dist-vertOffset)/(vertScale)-vertPixels);
		return pixel;
	}

	double dist(double x,double y,double x2,double y2) {
		double dist = sqrt(pow((x-x2), 2) + pow((y-y2), 2));
		return dist;
	}
};

float horizPixToDist(int pixel) {
	float dist = (horizScale*(pixel-(horizPixels/2)))+horizOffset;
	return dist;
}

int horizDistToPix(float dist) {
	float pixel = ((dist-horizOffset)/horizScale)+(horizPixels/2);
	return pixel;
}

float vertPixToDist(int pixel) {
	float dist = (vertScale*(vertPixels-pixel))+vertOffset;
	return dist;
}

int vertDistToPix(float dist) {
	float pixel = -((dist-vertOffset)/(vertScale)-vertPixels);
	return pixel;
}

int main(int argc, char** argv) {
	//Define the message handeler
	signal(SIGINT, intHandler);
	
	//Initialize the topic
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;

	//ROS Spin until we get a cntr+c
	ros::spin();
	
	//*Log Save Phase*//
	//Save all the images to files
	cout << "Saving images..." << endl;
	imwrite("LineTrackingFiles/frame.jpg", frame);
	imwrite("LineTrackingFiles/HSV.jpg", HSV);
	imwrite("LineTrackingFiles/threshld.jpg", threshld);
	imwrite("LineTrackingFiles/hough.jpg", ctv);
	imwrite("LineTrackingFiles/output.jpg", output);
	imwrite("LineTrackingFiles/can.jpg", Can);

	//Save text files of all of the points
	cout << "Saving points..." << endl;

	//Start the files
	ofstream pixels;
	ofstream points;
	pixels.open("LineTrackingFiles/pixel.txt");
	points.open("LineTrackingFiles/point.txt");

	//Run through the points and save them to a file
	for (size_t i = 0; i < lines.size(); i++) {
		//Get points for the array
		Vec4i l = lines[i];
		line(ctv, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, CV_AA);

		//Convert to meters
		float p0 = horizPixToDist(l[0]);
		float p1 = vertPixToDist(l[1]);
		float p2 = horizPixToDist(l[2]);
		float p3 = vertPixToDist(l[3]);

		//Save the points to a file
		pixels << l[1] << " , " << l[0] << "\r\n";
		points << p1 << " , " << p0 << "\r\n";
		pixels << l[3] << " , " << l[2] << "\r\n";
		points << p3 << " , " << p2 << "\r\n";
	}

	//Close the files
	pixels.close();
	points.close();

	//End the program
	return 0;
}
