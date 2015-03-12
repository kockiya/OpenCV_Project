#include "ros/ros.h"
#include "std_msgs/String.h"
#include "filteredObject.hpp"
#include <sstream>

using namespace std;
using namespace cv;

bool READ_OK = false;

void chatterCallback(const std_msgs::String::ConstPtr& s)
{
	ROS_INFO("Velocity reads: %s",s->data.c_str());
	READ_OK = true;
}

int main(int argc, char**argv)
{


	ros::init(argc, argv, "vPub");

	ros::NodeHandle n;

	ros::Publisher v_pub = n.advertise<std_msgs::String>("velocityVal", 1000);
	ros::Subscriber sub = n.subscribe("camCoords", 1000, chatterCallback);
	
	ros::Rate loop_rate(10);
    	std_msgs::String msg;

    	
  while (ros::ok())
  {

	ros::spinOnce();
	
     	if(READ_OK)
     	{
	    	string s =  "Velocity publishes: 'Some velocity value'";
		msg.data = "Some velocity value";
	
	    	v_pub.publish(msg);
	    	
	    	ROS_INFO("%s", s.c_str());
	    	READ_OK = false;
    	}
    

	loop_rate.sleep();
  }
  


return 0;

}
