/*
This node's role is as follows:

Subscriber to coord_pub
Publisher to cmd_vel

1) Click on point onscreen.
2) Get coordinate data again from coordinate publishing node.
	- Error if object is untrackable at this point. 
3) Do some calculations based on input and coordinate data to change orientatioe.
4) Publish information to robot's cmd_vel topic.

*/

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"
#include "filteredObject.hpp"
#include <sstream>
#include <iostream>
#include <vector>
//#include "opencv_coordinate_package/fObject.h"
//#include "opencv_coordinate_package/fObjectArray.h"
#include <deque>

using namespace std;
using namespace cv;

vector<string> split_str(const string &s, char delim);
vector<string> &split_str(const string &s, char delim, vector<string> &elems);
vector<float> calcSpeed(float target_x, float target_y, float k, float start_x, float start_y, float start_angle=0);
void clickDQPointCallBack(int event, int x, int y, int flags, void* userdata);

int NodeNumber;
float this_x;
float this_y;
float this_yaw;
bool this_can_track;

class RobotDriver
//Adapted code example
{
private:
  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  deque<clickPoint> clickPoints;
  vector<filteredObject> objects;
  

public:
  RobotDriver(ros::NodeHandle &nh)
  {
    nh_ = nh;
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void moveDistance(float distance, float speed, geometry_msgs::Twist t);
  
  //Move a certain distance foward in meters given distance and speed and then stop.
  
  
  bool driveInput()
  {
  
  	VideoCapture cam(0);
  	if (cam.isOpened()) 
  	{ 
		geometry_msgs::Twist base_cmd;
		Mat cameraFrame;
		
		cam.read(cameraFrame);
		imshow("cam", cameraFrame);
	
		if(restoreSettings(objects))
		{
			cout << "'config.json' found; filter settings loaded successfully!" << endl;
		}
		else
			cout << "'config.json' either not found or improperly formatted. Create new filters before trying this." << endl;
	
		//Specify orientation
		clickData cData;
		setMouseCallback("cam", click2PointCallBack, &cData);
	
		while(!cData.two_clicks)
		{
			waitKey(10);
			cam.read(cameraFrame);
			imshow("cam", cameraFrame);
			
			drawClickedData(cam, cData, cameraFrame);
			updateFilteredObjectPosition(cam, objects, -1);
			drawFilteredObject(cam, objects, cameraFrame, -1);
			for(int i = 0; i < clickPoints.size(); i++)
				drawClickedPoint(clickPoints[i], cameraFrame);
		}
		
		//Set yaw.
		float dist_spec = sqrt(pow((cData.x[0] - cData.x[1]),2) + pow((cData.y[0] - cData.y[1]),2));
		float dist_xaxis = sqrt(pow((cData.x[0] - cData.x[1]),2));
		
		this_yaw = acos(dist_xaxis/dist_spec);
		
		setMouseCallback("cam", clickDQPointCallBack, &clickPoints);
		
		
		
		while(nh_.ok())
		{
			waitKey(20);
			
			cam.read(cameraFrame);
			imshow("cam", cameraFrame);
			
			updateFilteredObjectPosition(cam, objects, -1);
			drawFilteredObject(cam, objects, cameraFrame, -1);
			for(int i = 0; i < clickPoints.size(); i++)
				drawClickedPoint(clickPoints[i], cameraFrame);
			
			
			if(!clickPoints.empty())
			{
			
			//Change orientation
			
			//Move
			
			clickPoints.pop_front();
			
			cam.read(cameraFrame);
			imshow("cam", cameraFrame);
			updateFilteredObjectPosition(cam, objects, -1);
			drawFilteredObject(cam, objects, cameraFrame, -1);
			for(int i = 0; i < clickPoints.size(); i++)
				drawClickedPoint(clickPoints[i], cameraFrame);
			}

 
    	}
    }
    else
    	cout << "ERROR: Could not connect to camera" << endl;
    return true;
    
  }

};
/*
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
	this_can_track = true;
	ROS_INFO("Subscriber see's object0: %s",s.c_str());
	}
	else
	{
		this_can_track = false;
		ROS_INFO("ERROR: Object ID %d was untrackable!",NodeNumber);
		return;
	}
}
*/
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

	//ros::Subscriber sub = n.subscribe("camCoords", 1000, chatterCallback);
	
	ros::Rate loop_rate(10);
	std_msgs::String msg;

	ROS_INFO("Input format: x y k");
	string input;
    	
	RobotDriver driver(n);
	driver.driveInput();
  


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





void RobotDriver::moveDistance(float distance, float speed, geometry_msgs::Twist t)
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

void clickDQPointCallBack(int event, int x, int y, int flags, void* userdata)
{
	deque<clickPoint>* cdq = (deque<clickPoint>*)userdata;
	if(event == EVENT_LBUTTONUP)
	{
		cdq->push_back(clickPoint(x, y));
	}
	if(event == EVENT_RBUTTONUP && !cdq->empty())
	{
		cdq->pop_back();
	}
}



