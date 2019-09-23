//BM TEST - Updated 5/2/2018

// copied from catkin_ws/src/simple_navigation_goals/src and renamed for new gps package - Ari 4/17/19
#include <ros/ros.h>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <tf/transform_datatypes.h>
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#define PI 3.14159265359

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*******************************************************************
 * Coordinates Struct to hold Lats and Longs
 *******************************************************************/
struct Coords
{
  float latitude;
  float longitude;
  float bearing;
  float hypot;
};

/*******************************************************************
 * Change the degrees input to radians
 *******************************************************************/
float degreesToRadians(float degrees) 
{
  return (degrees * (PI / 180));
}

/*******************************************************************
 * Get and return necessary bearing
 *******************************************************************/
float getBearing(float x, float y)
{
  return atan2(y,x) * (180/PI); 
}

/*****************************************************************************
 * Get the distance in meters between the current coordinates and 
 * 		the target coordinates
 * @param   
 * @return  The coords struct containing the waypoint data in meters 
******************************************************************************/
Coords distanceBetweenCoordinates(float nextLat, float nextLong, float currLat, float currLong) 
{
  Coords result;
  int earthRadiusm = 6371000;
  //float currLat = getCurrLatitude();
  //float currLong = getCurrLongitude();
  
  double dLat = degreesToRadians(nextLat-currLat);
  double dLon = degreesToRadians(nextLong-currLong);
 
  float lat1 = degreesToRadians(currLat);
  float lat2 = degreesToRadians(nextLat);
  float u = sin((dLat)/2);
  float v = sin((dLon)/2);
  float solution = 2.0 * earthRadiusm * asin(sqrt(u * u + cos(lat1) * cos(lat2) * v * v));
  printf("distance = %f\n", solution);

  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
  //float bearing = atan2(-x,y);//* (180/PI);
  //result.bearing = bearing;
  result.hypot = solution;
  //printf("prev bearing = %f\n", bearing*(180/PI));
  //bearing = atan2(-dLat,dLon);
  //printf("newcalc bearing = %f\n", bearing*(180/PI));
  float bearing = atan2(dLon,dLat);
  printf("newcalc2 bearing = %f\n", bearing*(180/PI));
  result.bearing = bearing;
  return result;
}

/*******************************************************************
 * Execute a command and get the result.
 * 
 * @param   cmd - The system command to run.
 * @return  The string command line output of the command.
 ******************************************************************/
std::string GetStdoutFromCommand(std::string cmd) 
{
  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1"); // Do we want STDERR?

  stream = popen(cmd.c_str(), "r");
  if (stream) 
  {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
  }
  return data;
}

/******************************************************************
 * Change string to float
 * 
 * @param   s - string to translate
 * @return  The float value indicated by s
 ******************************************************************/
float string_to_float( const std::string& s )
{
  std::istringstream i(s);
  float x;
  if (!(i >> x))
    return 0;
  return x;
} 

/******************************************************************
 * Get the Current Latitude from GPS
 * 
 * @return  The float value of the latitude
 ******************************************************************/
float getCurrLatitude()
{
  std::string latitude = GetStdoutFromCommand("rostopic echo -n 1 gps/fix/latitude");
  int spot = latitude.find("\n");
  latitude.replace(spot, 1, "\0");
  float lat = string_to_float(latitude);
  printf("Current LAT: %f\n",lat);
  return lat;
}

/******************************************************************
 * Get the Current Longitude from GPS
 * 
 * @return  The float value of the longitude
 ******************************************************************/
float getCurrLongitude()
{
  std::string longitude = GetStdoutFromCommand("rostopic echo -n 1 gps/fix/longitude");
  int spot = longitude.find("\n");
  longitude.replace(spot, 1, "\0");
  float lon = string_to_float(longitude);
  printf("Current LON: %f\n",lon);
  return lon;
}

/******************************************************************
 * Get the Current X value from Magnetometer
 * 
 * @return  The float value of X
 ******************************************************************/
float getMagX()
{
  std::string mag = "nan";
  while (!mag.compare("nan"))
  {
	mag = GetStdoutFromCommand("rostopic echo -n 1 imu/mag/magnetic_field/x");
    int spot = mag.find("\n");
    mag.replace(spot, mag.length()-spot, "\0");
    //std::cout << "Got String X: " << mag << "\n";
  } 
  float lon = string_to_float(mag);
  printf("Current Mag X: %f\n",lon);
  return lon;
}

/******************************************************************
 * Get the Current Y value from Magnetometer
 * 
 * @return  The float value of Y
 ******************************************************************/
float getMagY()
{
  std::string mag = "nan";
  while (!mag.compare("nan"))
  {
	mag = GetStdoutFromCommand("rostopic echo -n 1 imu/mag/magnetic_field/y");
    int spot = mag.find("\n");
    mag.replace(spot, mag.length()-spot, "\0");
    //std::cout << "Got String Y: " << mag << "\n";
  } 
  float lon = string_to_float(mag);
  printf("Current Mag Y: %f\n",lon);
  return lon;
}

/******************************************************************
 * Get the heading from Magnetometer
 * 
 * @return  The degree angle heading
 ******************************************************************/
float getMagDirection()
{
    float magX = getMagX();
    float magY = getMagY();
    float magDir = atan2(magY, magX) * (180/PI);
    printf("Current Mag Direction: %f\n",magDir);
	return magDir;
}

/******************************************************************
 * Get next latitude and longitude from GPS coords file
 * 
 * @params  line      - the line of coords to get
 *          latitude  - the pointer to latitude where 
 *                         the value will be written
 *          longitude - the pointer to longitude where the value goes
 ******************************************************************/
Coords getNextWaypoint(int line)
{
  std::ifstream coordFile("/home/ubuntu/Desktop/coordinates.txt");
  Coords returnVal;
  if(!coordFile) {
    std::cout << "Cannot open input file.\n";
    exit(1);
  }
  std::string x;
  int count = 0;
  while (count < line)
  {
    std::getline(coordFile, x);
    count++;
  }
  int spot = x.find(",");
  x.replace(spot, 1, "\0");
  returnVal.latitude = string_to_float(x);
  printf("Target LAT: %f\n", returnVal.latitude);
  returnVal.longitude = string_to_float(x.substr(spot+1));
  printf("Target LONG: %f\n", returnVal.longitude);
  coordFile.close();
  returnVal.bearing = 0;
  return returnVal;
}

/***************************************************************************
 * Split the next xy coordinates in order to get sub-waypoint
 * @param - 
 * @return - 
***************************************************************************/
Coords split(int splits, Coords next)
{
  Coords result;
  //float diffLat = nextLat - currLat;
  //float diffLong = nextLong - currLong;
  //diffLat = diffLat * 111111;
  //diffLong = diffLong * 111111;
  //result.latitude = diffLat / splits;
  //result.longitude = diffLong / splits;
  result.hypot = next.hypot / splits;
  //result.longitude = nextLong / splits;
  return result;
}

/*
void gpsSub(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO(msg->data.c_str);

}
*/

/******************************************************************
 * 10/13/2018
 ******************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  //ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("gps/fix", 100, gpsSub);
  int first = 1;
  int oriented = 0; // is the robot going the right way?
  int errorRange = .0001; 
  int line = 1;
  int count = 3; // the count on how many cuts until goal
  //tf::Quaternion q_orig, q_new;
  move_base_msgs::MoveBaseGoal goal;
  Coords currLocation; // the current location
  Coords nextWaypoint; // the next GPS waypoint
  Coords nextxy;       // the X/Y to next GPS waypoint
  Coords splitxy;      // the next split location to go to
	
  // Get Current Latitude and Longitude
  currLocation.latitude = getCurrLatitude(); // current latitude
  currLocation.longitude = getCurrLongitude(); // get current Longitude
  nextWaypoint = getNextWaypoint(line); // get the first line
	
  // was used for calibration
  while(1)
  {
    getMagDirection();
  }

  //GO FORWARD 5 METERS AND COMPARE GPS WAYPOINTS TO GET DIRECTION?

  //nextxy = distanceBetweenCoordinates(47.1187443, -88.5471899, 47.1190510, -88.5471899); // due south
  //nextxy = distanceBetweenCoordinates(47.1200510, -88.5471899, 47.1190510, -88.5461899); // due west
  //nextxy = distanceBetweenCoordinates(47.1200510, -88.5461899, 47.1200510, -88.5471899); // due east
  //nextxy = distanceBetweenCoordinates(47.1190510, -88.5461899, 47.1200510, -88.5471899); // due southeast
  //nextxy = distanceBetweenCoordinates(47.1190510, -88.5471899, 47.1187443, -88.5471899); // due north

  //we'll send a goal to the robot to move forward
  goal.target_pose.header.frame_id = "/map"; //"map"; //"base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  while(1)
  {   float temp = nextxy.bearing;
	  nextxy = distanceBetweenCoordinates(nextWaypoint.latitude, nextWaypoint.longitude, currLocation.latitude, currLocation.longitude); // get the distance (x,y) between here and the next gps waypoint
	  //splitxy = split(count, nextxy);

	  if (!oriented && first)
	  {
	    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -nextxy.bearing);
	    goal.target_pose.header.frame_id = "/map"; //"map"; //"base_link";
	    goal.target_pose.pose.orientation.w = 0;
		ROS_INFO_STREAM(q);
	    q.normalize();
	    quaternionTFToMsg(q, goal.target_pose.pose.orientation);
		goal.target_pose.pose.position.x = 0.5;
		first = 0;
	    //goal.target_pose.pose.orientation.z = nextxy.bearing;
	  }
	  else if (!oriented && !first)
	  {
	    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, -nextxy.bearing - temp);
	    goal.target_pose.header.frame_id = "/base_link"; //"map"; //"base_link";
	    goal.target_pose.pose.orientation.w = 0;
		ROS_INFO_STREAM(q);
	    q.normalize();
	    quaternionTFToMsg(q, goal.target_pose.pose.orientation);
		goal.target_pose.pose.position.x = 0.5;
	    //goal.target_pose.pose.orientation.z = nextxy.bearing;

	  }
	  else
	  {
		// craft a goal straight ahead
		goal.target_pose.header.frame_id = "/base_link";  
	    goal.target_pose.pose.position.x = nextxy.hypot;
		goal.target_pose.pose.position.y = 0;
        goal.target_pose.pose.orientation.z = 0;
	    goal.target_pose.pose.orientation.w = 1;
	  }


	  //ROS_INFO_STREAM(goal.target_pose.pose);
	  //goal.target_pose.pose.orientation.z = nextxy.bearing;
	  ROS_INFO_STREAM(goal);

	  ROS_INFO("Sending goal");
	  ac.sendGoal(goal);

	  ac.waitForResult();

	  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	  {
	    ROS_INFO("SUCCESS");
	    if (!oriented)
		{
		  oriented = 1; 
		}
		else
		{
		  oriented = 0;
		}
	    count--;
		// then get next coordinates
	    if (((getCurrLatitude() <= nextWaypoint.latitude + errorRange) && (getCurrLatitude() >= nextWaypoint.latitude - errorRange)) || count == 0)
	    {
	      if (((getCurrLongitude() <= nextWaypoint.longitude + errorRange) && (getCurrLongitude() >= nextWaypoint.longitude - errorRange)) || count == 0)
	      {
			ROS_INFO("GOT TO WAYPOINT");
			count = 3; //reset number of splits to make
			//oriented = 0;
			line++;
			if (getNextWaypoint(line).latitude == 0)
			{
			  ROS_INFO("RUN COMPLETE!");
			  break;
			}
			nextWaypoint = getNextWaypoint(line);
			// get the distance (x,y) between here and the next gps waypoint
			//nextxy = distanceBetweenCoordinates(nextWaypoint.latitude, nextWaypoint.longitude, currLocation.latitude, currLocation.longitude); 
	      }
	    }
	  }
	  else
	  {
		ROS_INFO("FAILURE");
	    // recalculate course (try straight x, then straight y?)
	  }

	  currLocation.latitude = getCurrLatitude();
  	  currLocation.longitude = getCurrLongitude();
  }

  return 0;
}
