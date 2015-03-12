#include "ros/ros.h"
#include "filteredObject.hpp"
#include "std_msgs/String.h"

using namespace std;
using namespace cv;


void chatterCallback(const std_msgs::String::ConstPtr& s)
{

	ROS_INFO("Test vSubscriber sees: %s",s->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "vlistener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("velocityVal", 1000, chatterCallback);

  ros::spin();

  return 0;
}
