#include "ros/ros.h"
#include "filteredObject.hpp"
#include "std_msgs/String.h"
#include "opencv_coordinate_package/fObject.h"
#include "opencv_coordinate_package/fObjectArray.h"

using namespace std;
using namespace cv;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const opencv_coordinate_package::fObjectArray::ConstPtr& o)
{
	//Need to look into getting argv[1] so that the object 'ID' can be specified so
	//subscribers only pay attention to o.objects[ID]
	
	string s = "name: " + o->objects[0].name + " X: " + to_string(o->objects[0].x) + " Y: " + to_string(o->objects[0].y);
	ROS_INFO("Subscriber see's object0: %s",s.c_str());
	
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener"); //'listener' = name of this node
  ros::NodeHandle n;

  //'camCoords' = name of the topic to subscribe
  //chatterCallback = name of function to call
  ros::Subscriber sub = n.subscribe("camCoords", 1000, chatterCallback); 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
