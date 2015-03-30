#include "ros/ros.h"
#include "std_msgs/String.h"
#include "opencv_coordinate_package/fObject.h"
#include "opencv_coordinate_package/fObjectArray.h"
#include "filteredObject.hpp"
#include <sstream>

using namespace std;
using namespace cv;

int main(int argc, char**argv)
{

	const bool show_camera = true;
	
	ros::init(argc, argv, "coordPub");


	ros::NodeHandle n;

	//advertise<type_of_msg_to_return>
	ros::Publisher coord_pub = n.advertise<opencv_coordinate_package::fObjectArray>("camCoords", 1000);
	
	ros::Rate loop_rate(20);
	
	
	
	VideoCapture cam(0);
	namedWindow("cam", 1);
	Mat cameraFrame, hsvFrame, maskFrame, maskErode;
	vector<filteredObject> objects;
    if(restoreSettings(objects))
    {
    	ROS_INFO("%s", "coordPub successfully loaded 'config.json'");
    }
    	
    else
    {
    	ROS_INFO("%s", "WARNING: coordPub could not load config.json. \n*** Please find or create new filters using the filter script!");
    }
    	
    
    
    //std_msgs::String msg;
    
    opencv_coordinate_package::fObjectArray msg;
    cam.read(cameraFrame);
    
    msg.size = objects.size();
    for(int i = 0; i < msg.size; i++)
    //Initialize message array to correspond with filteredObject vector.
    {
    	opencv_coordinate_package::fObject f;
    	f.name = objects[i].name;
    	f.x = objects[i].x*objects[i].scale;
    	f.y = objects[i].y*objects[i].scale;
    	msg.objects.push_back(f);
    }
    
    
    bool atleastOneTracked = false;
    bool show_once = true;
  while (ros::ok())
  {

   	waitKey(5);
   	for(int i = 0; i < objects.size(); i++)
   	{
   		configureMasks(cameraFrame, hsvFrame, maskFrame, maskErode, objects, i);
		if(!(drawFilteredObject(cam, objects, cameraFrame, i) && updateFilteredObjectPosition(cam, objects, i)))
		{
			ROS_INFO("%s", "ERROR: Could not gather data from camera!");
			break;
		}
		
		if(objects[i].can_track)
		{
			atleastOneTracked = true;
			//TODO: Add can_track to fObject.msg 
			msg.objects[i].x = objects[i].x*objects[i].scale;
			msg.objects[i].y = objects[i].adjusted_y*objects[i].scale;
			string z = "name: " + msg.objects[i].name + " X: " + to_string(msg.objects[i].x) + " Y: " + to_string(msg.objects[i].y);
			ROS_INFO("%s", z.c_str());
			
		}

		
		msg.objects[i].can_track = objects[i].can_track;
		
	}
	//if(show_camera)
	
	if(atleastOneTracked)
	//Publishes information about all nodes if atleast one object was being tracked.
	{
		coord_pub.publish(msg);
		atleastOneTracked = false;
		
		
		string s = "This node (coordPub) is publishing coordinate info to topic 'camCoord'.";
		
		if(show_once)
			ROS_INFO("%s", s.c_str());
		show_once = show_once ? !show_once : show_once;
	}
	
	imshow("cam", cameraFrame);
	cam.read(cameraFrame);
	
	
    

    ros::spinOnce();

    loop_rate.sleep();
  }


return 0;

}
