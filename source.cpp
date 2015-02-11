//Author: kockiya
//Description: Small program put together for filter calibration for simple color-code motion tracking with opencv. Compiled using g++ 4.9, opencv 2.4.8, jsoncons release 0.96

/*
g++ -std=c++11 -I . -I/usr/local/include/opencv -I/usr/local/include/opencv2 -L/usr/local/lib/ -g -o binary  source.cpp -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching
*/

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <unistd.h>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

//All credits for JSON related operations go to github.com/danielaparker for their easy-to-use header-only JSON processing library.
#include "jsoncons/json.hpp"

using namespace cv;
using namespace std;
using jsoncons::json;

class filteredObject{
//A filter object contains the various settings required to produce a nice filtered binary image.
	public:
	string name;
	vector<int> lower;
	vector<int> upper;
	Mat ekernel;
	Mat dkernel;
	Mat mask;
	Moments moment;
	int x, y, ekernel_val, dkernel_val;
	bool can_track;
	double area;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	
	filteredObject(){
		name = "object";
		lower.push_back(0); lower.push_back(0); lower.push_back(0);
		upper.push_back(0); upper.push_back(0); upper.push_back(0);
		dkernel.create(1, 1, CV_8UC1);
		ekernel.create(1, 1, CV_8UC1);
		ekernel.setTo(Scalar(1));
		dkernel.setTo(Scalar(1));
		ekernel_val = dkernel_val = 1;
		can_track = false;
	}
	
	void setName(string s){
		name = s;
	}
	
	//For debugging purposes; not needed normally as sliders modify member values directly.
	void setValues(int lh, int ls, int lv, int uh, int us, int uv, int d, int e){
	lower[0] = lh; lower[1] = ls; lower[2] = lv;
	upper[0] = uh; upper[1] = us; upper[2] = uv;
	dkernel.create(d, d, CV_8UC1);
	ekernel.create(e, e, CV_8UC1);
	ekernel.setTo(Scalar(1));
	dkernel.setTo(Scalar(1));
	}
	
	
};

//Creates the sliders used for filtering.
void create_sliders(filteredObject* &f);


void changeKernel(int i, void* v);
void saveSettings(vector<filteredObject> f);
void restoreSettings(vector<filteredObject> &objects);


int main() {
	VideoCapture cam(0);
	int keycode, counter = 0; 
	Mat cameraFrame, hsvFrame, maskFrame, maskErode;
	vector<filteredObject> objects;
	filteredObject* selected_object;
	namedWindow("mask", 1); namedWindow("cam", 1); namedWindow("sliders",1);
	
	bool config_exists = false;
	ifstream fin("config.json");
	if(fin)
	{
		config_exists = true;
		fin.close();
	}
		
	if(config_exists)
	{
		restoreSettings(objects);
		cout << "'config.json' found; filter settings loaded successfully!" << endl;
	}
	

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
		
			for(int i = 0; i < objects.size(); i++)
			{
				//Convert camera frame from RBG to HSV (hue, saturation value/brightness);
				cvtColor(cameraFrame, hsvFrame, CV_BGR2HSV);
				
				//Turn the HSV frame into a binary mask based on lower/uper bound HSV limits.
				inRange(hsvFrame, objects[i].lower, objects[i].upper, maskFrame);
			
				//Get rid of "specs" and small noise; size to get rid of is specified by the ekernel size;
				erode(maskFrame, maskErode, objects[i].ekernel);
			
				//Enlarge current points by a size specified by dkernel
				dilate(maskErode, objects[i].mask, objects[i].dkernel);
				
				//Show the filter currently being configured on the "mask" window
				if(selected_object == &objects[i]) 
					imshow("mask", selected_object->mask);
				
				keycode = waitKey(20);
				
				//Finds all contours (edges/outlines) from the mask, saves to contours/hierarchy vectors
				findContours(objects[i].mask, objects[i].contours, objects[i].hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
				if(objects[i].hierarchy.size() > 0 && objects[i].contours.size() > 0){
					for(int j = 0; j >= 0; j = objects[i].hierarchy[j][0]){
					//Credits for finding x, y, moments and area goes to youtube.com/khounslow as I do 
					//not understand what image moments are (computer vision related concept).
							objects[i].moment = moments((Mat)objects[i].contours[j]);
							objects[i].area = objects[i].moment.m00;
					
							if(objects[i].area > 400)
							{
								objects[i].x = objects[i].moment.m10/objects[i].area;
								objects[i].y = objects[i].moment.m01/objects[i].area;
								objects[i].can_track = true;
							}
							else
								objects[i].can_track = false;
						}
					}
					else
						objects[i].can_track = false;
					
					if(objects[i].can_track)
					{
						//Draw a circle if the found largest contour is big enough (can_track);
						int radius = (int)sqrt(objects[i].area/3.14);
						const int xshift = 10, yshift = 10, center_size = 4;
						circle(cameraFrame, Point(objects[i].x, objects[i].y), radius , Scalar(0, 255, 0), 2);
						circle(cameraFrame, Point(objects[i].x, objects[i].y), center_size, Scalar(0, 255, 0), -1);
						radius += 10;
						
						line(cameraFrame, Point(objects[i].x, objects[i].y), Point(objects[i].x+xshift, objects[i].y+radius), Scalar(0, 255, 0)); 
						
						
						putText(cameraFrame, objects[i].name, Point(objects[i].x+xshift, objects[i].y+radius), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
						putText(cameraFrame, "X: "+to_string(objects[i].x) , Point(objects[i].x+xshift, objects[i].y+yshift+radius), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
						putText(cameraFrame, "Y: "+to_string(objects[i].y) , Point(objects[i].x+xshift, objects[i].y+2*yshift+radius), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
						
					}
			
			}
			
		
			imshow("cam", cameraFrame);
			cam.read(cameraFrame);
		
			//27 = ESC, 115 = 'S'
			if(keycode == 27)
				break;
			else if(keycode == 115)
			{
				saveSettings(objects);
				
				filteredObject f;
				objects.push_back(f);
				selected_object = &objects[objects.size()-1];
				
				create_sliders(selected_object);
				
			}
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


void saveSettings(vector<filteredObject> f)
{
	json file_export;
	json* entry = new json[f.size()];
	
	for(int i = 0; i < f.size(); i++)
	{
		if(f[i].name == "object")
			f[i].name = "object"+to_string(i);
		
		entry[i]["hueUp"] = f[i].upper[0];
		entry[i]["satUp"] = f[i].upper[1];
		entry[i]["valUp"] = f[i].upper[2];
		entry[i]["hueLow"] = f[i].lower[0];
		entry[i]["satLow"] = f[i].lower[1];
		entry[i]["valLow"] = f[i].lower[2];
		entry[i]["ekernel"] = f[i].ekernel_val;
		entry[i]["dkernel"] = f[i].dkernel_val;
		
		file_export[f[i].name] = entry[i];
	}
	
	ofstream out("config.json");
	
	out << file_export << endl;
	
	out.close();
	
	cout << "Settings saved to config.json" << endl;
}

void restoreSettings(vector<filteredObject> &objects)
{
	json items = json::parse_file("config.json");
	for(auto it = items.begin_members(); it != items.end_members(); it++)
	{
	    filteredObject f;
	    json& entry = it->value();
	    
	    f.name = it->name();
	    f.upper[0] = entry["hueUp"].as<int>();
		f.upper[1] = entry["satUp"].as<int>();
		f.upper[2] = entry["valUp"].as<int>();
		f.lower[0] = entry["hueLow"].as<int>();
		f.lower[1] = entry["satLow"].as<int>();
		f.lower[2] = entry["valLow"].as<int>();
		f.ekernel_val = entry["ekernel"].as<int>();
		f.dkernel_val = entry["dkernel"].as<int>();
		
		objects.push_back(f);

	}
}

