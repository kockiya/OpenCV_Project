//Author: kockiya
//Description: Small program put together for filter calibration for simple color-code motion tracking with opencv. Compiled using a cmake script, opencv 2.4.8, jsoncons release 0.96

/*
 -- Uses the opencv_coordinate_package CMakeLists.txt to compile. (It was the easiest way to get bag files to work)
*/

#include "ros/ros.h"
#include <unistd.h>
#include "filteredObject.hpp"
#include <cmath>
#include <rosbag/bag.h>
#include <sys/time.h>
#include "opencv_coordinate_package/fStat.h"

//All credits for JSON related operations go to github.com/danielaparker for their easy-to-use header-only JSON processing library.


using namespace cv;
using namespace std;


//Creates the sliders used for filtering.
void create_sliders(filteredObject* &f);
void changeKernel(int i, void* v);
void clickPointCallBack(int event, int x, int y, int flags, void* userdata);

//TODO:
//Create feature that uses distance between clicked points to change scaling.
//Create feature that uses clicked points to change origin.

int main() {
	rosbag::Bag b;
	opencv_coordinate_package::fStat pointData;
	ros::Time::init();
	
	
	
	timeval curTime;
    	gettimeofday(&curTime, NULL);
    	int milli = curTime.tv_usec / 1000;
    	char buffer [80];
    	char currentTime[84] = "";
	struct tm * timer;
	time_t rawtime;
	time(&rawtime);
	timer = localtime(&rawtime);
	
	VideoCapture cam(0);
	int keycode, counter = 0;
	string filename = "movedata.txt", data_buffer = "";
	bool output_data = false;
	bool config_loaded = false;
	Mat cameraFrame, hsvFrame, maskFrame, maskErode;
	vector<filteredObject> objects;
	filteredObject* selected_object;
	namedWindow("mask", 1); namedWindow("cam", 1); namedWindow("sliders",1);
	float scale = 1;
	
	clickData cData;
	
	
	if(restoreSettings(objects))
	{
		cout << "'config.json' found; filter settings loaded successfully!" << endl;
		config_loaded = true;	
	}
	else
		cout << "'config.json' either not found or improperly formatted. Create new filters." << endl;
	

	if (cam.isOpened()) { 
		cout << "Camera opened successfully, hold ESC to close." << endl;
		cout << " -- Press S to save current filter to file and to configure new filter" << endl;
		cout << " -- Press W to toggle saving data to file" << endl;
		cout << " -- Press X to change coordinate scale" << endl;
		cam.read(cameraFrame);
		imshow("cam", cameraFrame);


		filteredObject f;
		objects.push_back(f);
		selected_object = &objects[objects.size()-1];
		
		create_sliders(selected_object);
		
		setMouseCallback("cam", clickPointCallBack, &cData);

		while(true)
		{
			//Note: Keycodes are platform dependent.
			//keycodes on Ubuntu on Axiom start with 1048 
			keycode = waitKey(20);
			if(keycode == 27 || keycode == 1048603)
				break;
			for(int i = 0; i < objects.size(); i++)
			{
				
				configureMasks(cameraFrame, hsvFrame, maskFrame, maskErode, objects, i);
				if(selected_object == &objects[i]) 
					imshow("mask", selected_object->mask);
				
				if(keycode == 27 || keycode == 1048603)
					break;
				
				//Finds all contours (edges/outlines) from the mask, saves to contours/hierarchy vectors
				//Draws on camera frame.
				
				updateFilteredObjectPosition(cam, objects, i);
				drawFilteredObject(cam, objects, cameraFrame, i);
				drawClickedPoints(cam, cData, cameraFrame);
				
				if(objects[i].can_track && output_data)
				{
				
					
					//Bag file should handle timestamp.
					
					pointData.id = i;
					pointData.x = objects[i].adjusted_x;
					pointData.y = objects[i].adjusted_y;
					
					b.write("pointData", ros::Time::now(), pointData);

				}
				
			
			}
			
		
			imshow("cam", cameraFrame);
			cam.read(cameraFrame);
		
			//27 = ESC, 115 = 'S'
			if(keycode == 27 || keycode == 1048603)
				break;
			else if(keycode == 115 || keycode == 1048691)
			{
				saveSettings(objects);
				
				filteredObject f;
				objects.push_back(f);
				selected_object = &objects[objects.size()-1];
				
				create_sliders(selected_object);
				
			}
			else if(keycode == 1048695) //W
			{
				output_data = !output_data;
				if(output_data)
				{
				putText(cameraFrame, "(The terminal requires your input)" , Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				putText(cameraFrame, "(The terminal requires your input)" , Point(30, 450), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				imshow("cam", cameraFrame);
				waitKey(20);
				cout << "Enter the new filename that will contain the data: ";
				cin >> filename;
				cout << endl;
				
				b.open(filename, rosbag::bagmode::Write);
				}
				else
				{
				b.close();
				}

			}
			else if(keycode == 1048696) //"X" on Axiom.
			{
				putText(cameraFrame, "(The terminal requires your input)" , Point(30, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				putText(cameraFrame, "(The terminal requires your input)" , Point(30, 450), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0));
				imshow("cam", cameraFrame);
				waitKey(20);
				string choice = "";
				cout << "\n\nIn order to change coordinate scale, you need to configure the mask which will \n\tact as reference object. Make sure the current filter has \n\tbeen calibrated with the sliders for a clear white-on-black image." << endl;
				cout << "You are about to change the coordinate scale, continue? (y/n): ";
				cin >> choice;
				if(choice == "y" || choice == "Y")
				{
					float real_diameter = 0;
					cout << "\nWhat is the real diameter of the **currently masked** object?:";
					cin >> real_diameter;
					scale = real_diameter/(selected_object->radius*2);
					changeScale(objects, scale);
					cout << "\nThe new scale is has been set! Save the new scale to file? (y/n): ";
					string choice2 = "";
					cin >> choice2;
					if(choice2 == "y" || choice2 == "Y")
					{
						saveSettings(objects, true);
					}
					cout << "\n";
				}
				
			}
			
			
		}
		
	}
	else
	{
		cout << "Could not open the camera! " << endl;
	}
	
	if(output_data)
	{
	/*
		if(counter > 0)
		{
			out << data_buffer;
			
		}
	*/
		counter = 0;
		data_buffer = "";
		b.rosbag::Bag::close();
	}
	
	cout << "(The program  has closed)" << endl;
	
	
	return 0;
	
}

void changeKernel(int i, void *v)
{
	Mat* kernel = (Mat*)v;
	kernel->create(i, i, CV_8UC1);
	kernel->setTo(Scalar(1));

}

void create_sliders(filteredObject* &selected_object)
{
	createTrackbar("Upper H", "sliders", &selected_object->upper[0], 180);
	createTrackbar("Lower H", "sliders", &selected_object->lower[0], 180);

	createTrackbar("Upper S", "sliders", &selected_object->upper[1], 255);
	createTrackbar("Lower S", "sliders", &selected_object->lower[1], 255);

	createTrackbar("Upper V", "sliders", &selected_object->upper[2], 255);
	createTrackbar("Lower V", "sliders", &selected_object->lower[2], 255);
	
	createTrackbar("Erode", "sliders", &selected_object->ekernel_val, 25, changeKernel, &selected_object->ekernel);
	createTrackbar("Dilate", "sliders", &selected_object->dkernel_val, 25, changeKernel, &selected_object->dkernel);
	
	

}

void clickPointCallBack(int event, int x, int y, int flags, void* userdata)
{
//Puts screen coordinates of click information into clickData struct. Limited to
//storing two coordinates at a time; should alternate between stored points 0 and 1.
	
	if(event == EVENT_LBUTTONUP)
	{
		clickData* cd = (clickData*)userdata;
		int clicked_count = cd->clicked_count;

		if(cd->clicked_count <= 1)
		{
			cd->x[clicked_count] = x;
			cd->y[clicked_count] = y;
			cd->clicked_count++;
			cd->clicked_count = cd->clicked_count <= 1 ? cd->clicked_count : 0;
		
		}

	}
	
}




