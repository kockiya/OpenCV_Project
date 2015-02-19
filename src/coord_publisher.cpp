#include "ros/ros.h"
#include "std_msgs/String.h"
#include "filteredObject.hpp"
#include <sstream>

using namespace std;
using namespace cv;

int main(int argc, char**argv)
{

	const bool show_camera = true;
	
	ros::init(argc, argv, "coordPub");


	ros::NodeHandle n;


	ros::Publisher coord_pub = n.advertise<std_msgs::String>("camCoords", 1000);
	
	ros::Rate loop_rate(10);
	
	
	
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
    	
    
    
    std_msgs::String msg;
    
    cam.read(cameraFrame);
    	
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
			string s =  "Publisher writes: " + objects[i].name + " - X:" + to_string(objects[i].x) + " Y:" + to_string(objects[i].y);
			msg.data = s;
			ROS_INFO("%s", s.c_str());
			coord_pub.publish(msg);
		}
		
	}
	//if(show_camera)
	
	imshow("cam", cameraFrame);
	cam.read(cameraFrame);
	
	

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    

    ros::spinOnce();

    loop_rate.sleep();
  }


return 0;

}
