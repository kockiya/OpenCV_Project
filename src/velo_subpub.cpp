/*
This node's role is as follows:

Subscriber to coord_pub
Publisher to cmd_vel

1) Prompt for user input.
	- User should input target_x, target_y, and k.
2) Get coordinate data again from coordinate publishing node.
	- Error if object is untrackable at this point. 
3) Do some calculations based on input and coordinate data.
4) Publish information to robot's cmd_vel topic.

*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "filteredObject.hpp"
#include <sstream>
#include <iostream>
#include <vector>
#include "opencv_coordinate_package/fObject.h"
#include "opencv_coordinate_package/fObjectArray.h"

using namespace std;
using namespace cv;

vector<string> split_str(const string &s, char delim);
vector<string> &split_str(const string &s, char delim, vector<string> &elems);
vector<float> calcSpeed(float target_x, float target_y, float k, float start_x, float start_y, float start_angle=0);

int NodeNumber;
float this_x;
float this_y;


class RobotDriver
//Adapted code example
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void moveDistance(float distance, float speed, geometry_msgs::Twist t)
  //Move a certain distance foward in meters given distance and speed and then stop.
  {
  	timeval start;
  	timeval end;
    	gettimeofday(&end, NULL);
    	gettimeofday(&start, NULL);
    	int startTime = concatenate_int(start.tv_sec, start.tv_usec/1000);
    	int endTime = concatenate_int(end.tv_sec, end.tv_usec/1000);
    	
    	
	//v = d/t -> t = d/v
	int totalTime = (distance/speed)*1000;
	
	
	cmd_vel_pub_.publish(t);
	
	while((endTime - startTime) < totalTime)
	{
	    gettimeofday(&end, NULL);
	    endTime = concatenate_int(end.tv_sec, end.tv_usec/1000);
	    cmd_vel_pub_.publish(t);
	    
	}
	
	//TODO: Smooth stop instead of abrupt.
	geometry_msgs::Twist stop_t;
	stop_t.linear.x = 0;
	cmd_vel_pub_.publish(stop_t);
  }
  
  //! Loop forever while sending drive commands based on keyboard input
  bool driveInput()
  {
    cout << "Type in the distance and speed in meters. \n  EX: 0.5 0.1\t\tGoes 0.5 meters at 0.1m/s\n(Maximum 5 meters, 0.7 m/s)\n";


    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    string input;
    while(nh_.ok()){

      cout << "Enter command: ";
      getline(cin, input);
      vector<string> input_val = split_str(input, ' ');
      float dist, spd;
      
      //ros::spinOnce();
      
      if(input_val.size() != 2 || stof(input_val[0]) > 5 || stof(input_val[1]) > 0.7)
      {
        cout << "ERROR: Invalid command; must give valid distace and speed" << endl;
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      
      try{
      dist = stof(input_val[0]);
      spd = stof(input_val[1]);
      base_cmd.linear.x = spd;
      }
      catch(...)
      {
    	cout << "ERROR: Invalid input values. Must be numbers." << endl;
      }
      
      moveDistance(dist, spd, base_cmd);
      
 
    }
    return true;
  }

};

void chatterCallback(const opencv_coordinate_package::fObjectArray::ConstPtr& o)
{

	
	if(NodeNumber >= o->size || NodeNumber < 0)
	{
		ROS_INFO("ERROR: Object ID %d doesn't exist! (Is the overhead camera node working?)",NodeNumber);
		return;
		
	} 
	if(o->objects[NodeNumber].can_track)
	{
	this_x = o->objects[NodeNumber].x;
	this_y = o->objects[NodeNumber].y;
	string s = "name: " + o->objects[NodeNumber].name + " X: " + to_string(o->objects[NodeNumber].x) + " Y: " + to_string(o->objects[NodeNumber].y);
	ROS_INFO("Subscriber see's object0: %s",s.c_str());
	}
	else
	{
		ROS_INFO("ERROR: Object ID %d was untrackable!",NodeNumber);
		return;
	}
}

int main(int argc, char**argv)
{

	if(argc < 2)
	{
	ROS_INFO("ERROR: Must include object ID! \n\t\tEXAMPLE: rosrun opencv_coordinate_package vsubpub 0\n");
	return 0;
	}
	string nodeNumStr(argv[1]);
	NodeNumber = stoi(nodeNumStr);
	string node_name = "vpub" + nodeNumStr;
	ros::init(argc, argv, node_name.c_str()); //'listener #' = name of this node
	ros::NodeHandle n;


	ros::Publisher v_pub = n.advertise<std_msgs::String>("velocityVal", 1000);
	ros::Subscriber sub = n.subscribe("camCoords", 1000, chatterCallback);
	
	ros::Rate loop_rate(10);
    	std_msgs::String msg;

    	ROS_INFO("Input format: x y k");
    	string input;
    	
  while (ros::ok())
  {
	RobotDriver driver(n);
	driver.driveInput();
	
	/*
	cout << "Enter the target coordinates:" << endl; 
   	getline(cin, input);
	vector<string> params = split_str(input, ' ');
	
	

    	string s =  "Velocity publishes: 'Some velocity value'";
	msg.data = "Some velocity value";

    	v_pub.publish(msg);
    	
    	ROS_INFO("%s", s.c_str());


	ros::spinOnce();
	
	
	loop_rate.sleep();
	*/
  }
  


return 0;

}

vector<float> calcSpeed(float target_x, float target_y, float k, float start_x, float start_y, float start_angle)
{
	//ROS_INFO("RCVD REQ: FROM (%d, %d, %d)\n\t to (%d, %d) with k=%d", start_x, start_y, start_angle, target_x, target_y, k);
	float delta_y = target_y - start_y;
	float delta_x = target_x - start_x;
	float distance = sqrt(pow(delta_y,2) - pow(delta_x,2));
	float new_angle = atan(delta_y/delta_x);
	float new_speed = distance/k;
	
	vector<float> result;
	result.push_back(new_speed);
	result.push_back(new_angle);
	return result;
	
}










