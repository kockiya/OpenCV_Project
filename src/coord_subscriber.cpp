#include "ros/ros.h"
#include "filteredObject.hpp"
#include "std_msgs/String.h"
#include "opencv_coordinate_package/fObject.h"
#include "opencv_coordinate_package/fObjectArray.h"

using namespace std;
using namespace cv;


int NodeNumber;

void chatterCallback(const opencv_coordinate_package::fObjectArray::ConstPtr& o)
{
	//Need to look into getting argv[1] so that the object 'ID' can be specified so
	//subscribers only pay attention to o.objects[ID]
	
	if(NodeNumber >= o->size || NodeNumber < 0)
	{
		ROS_INFO("ERROR: Object ID %d doesn't exist!",NodeNumber);
		return;
		
	} 
	if(o->objects[NodeNumber].can_track)
	{
	string s = "name: " + o->objects[NodeNumber].name + " X: " + to_string(o->objects[NodeNumber].x) + " Y: " + to_string(o->objects[NodeNumber].y);
	ROS_INFO("Subscriber see's object0: %s",s.c_str());
	}
	
}


int main(int argc, char **argv)
{

  if(argc < 2)
  {
  ROS_INFO("ERROR: Must include object ID! \n\t\tEXAMPLE: rosrun opencv_coordinate_package listener 0\n");
  return 0;
  }
  string nodeNumStr(argv[1]);
  NodeNumber = stoi(nodeNumStr);
  string node_name = "listener" + nodeNumStr;
  ros::init(argc, argv, node_name.c_str()); //'listener #' = name of this node
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
