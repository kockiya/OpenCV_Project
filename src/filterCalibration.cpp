//Author: kockiya
//Description: Small program put together for filter calibration for simple color-code motion tracking with opencv. Compiled using g++ 4.9, opencv 2.4.8, jsoncons release 0.96

/*
g++ -std=c++11 -I . -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o filterCalibration  filterCalibration.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching
*/


#include <unistd.h>
#include "filteredObject.hpp"
#include <cmath>

//All credits for JSON related operations go to github.com/danielaparker for their easy-to-use header-only JSON processing library.


using namespace cv;
using namespace std;


//Creates the sliders used for filtering.
void create_sliders(filteredObject* &f);
void changeKernel(int i, void* v);

int main() {
	VideoCapture cam(0);
	int keycode, counter = 0; 
	Mat cameraFrame, hsvFrame, maskFrame, maskErode;
	vector<filteredObject> objects;
	filteredObject* selected_object;
	namedWindow("mask", 1); namedWindow("cam", 1); namedWindow("sliders",1);
	int debugcounter = 0;
	if(restoreSettings(objects))
		cout << "'config.json' found; filter settings loaded successfully!" << endl;
	else
		cout << "'config.json' either not found or improperly formatted. Create new filters." << endl;
	

	if (cam.isOpened()) { 
		cout << "Camera opened successfully, hold ESC to close." << endl;
		cout << " -- Press S to save current filter to file and to configure new filter" << endl;
		cam.read(cameraFrame);
		imshow("cam", cameraFrame);


		filteredObject f;
		objects.push_back(f);
		selected_object = &objects[objects.size()-1];
		
		create_sliders(selected_object);

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
				//Draws on camera.
				
				updateFilteredObjectPosition(cam, objects, i);
				drawFilteredObject(cam, objects, cameraFrame, i);
			
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
			//debugcounter++;
			//cout << "Debug Counter: " << debugcounter  << " Keycode: " << keycode << endl;
			if(debugcounter > 500)
				break;
		}
		
	}
	else
	{
		cout << "Could not open the camera! " << endl;
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




