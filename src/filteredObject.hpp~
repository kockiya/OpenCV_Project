#ifndef FILTEREDOBJECT_HPP_
#define FILTEREDOBJECT_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "jsoncons/json.hpp"

using namespace std;
using namespace cv;
using jsoncons::json;

class filteredObject{
//A filter object contains the various settings required to produce a nicely filtered binary image.
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

bool restoreSettings(vector<filteredObject> &objects)
{

	bool config_exists = false;
	ifstream fin("config.json");
	if(fin)
	{
		config_exists = true;
		fin.close();
	}
		
	if(config_exists)
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
		return true;
	}
	return false;
}

//Most of these functions could look prettier if they were recursive, but I'm too
//lazy and want something that just works for now. For the most part, if i or z
//is negative (by default) then these functions apply to all objects in the 
//vector ... otherwise it applies to only the specified index.


void configureMasks(Mat &cameraFrame, Mat &hsvFrame, Mat &maskFrame, Mat &maskErode, vector<filteredObject> &objects, int i=-1)
{
	if(i <= -1)
	{
		for(int k = 0; k < objects.size(); k++)
		{
		//Convert camera frame from RBG to HSV (hue, saturation value/brightness);
		cvtColor(cameraFrame, hsvFrame, CV_BGR2HSV);
	
		//Turn the HSV frame into a binary mask based on lower/uper bound HSV limits.
		inRange(hsvFrame, objects[i].lower, objects[i].upper, maskFrame);

		//Get rid of "specs" and small noise; size to get rid of is specified by the ekernel size;
		erode(maskFrame, maskErode, objects[i].ekernel);

		//Enlarge current points by a size specified by dkernel
		dilate(maskErode, objects[i].mask, objects[i].dkernel);
		}
	}
	else
	{
		//Convert camera frame from RBG to HSV (hue, saturation value/brightness);
		cvtColor(cameraFrame, hsvFrame, CV_BGR2HSV);
	
		//Turn the HSV frame into a binary mask based on lower/uper bound HSV limits.
		inRange(hsvFrame, objects[i].lower, objects[i].upper, maskFrame);

		//Get rid of "specs" and small noise; size to get rid of is specified by the ekernel size;
		erode(maskFrame, maskErode, objects[i].ekernel);

		//Enlarge current points by a size specified by dkernel
		dilate(maskErode, objects[i].mask, objects[i].dkernel);
	}
	
}

bool updateFilteredObjectPosition(VideoCapture &cam, vector<filteredObject> &objects, int z = -1)
{
	if(!cam.isOpened())
		return false;
		
	
				
	if(z > -1)
	{
	int i = z;
		
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
					
	}
	else
	{
	for(int i = 0; i < objects.size(); i++)
	{
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
		}
	
	}	
					return true;
}

bool drawFilteredObject(VideoCapture &cam, vector<filteredObject> &objects, Mat &cameraFrame, int z = -1)
{
	if(!cam.isOpened())
		return false;
	
	if(z > -1)
	{
	int i = z;
	
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
		else
		{
			for(int i = 0; i < objects.size(); i++)
			{
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

	return true;

}


#endif
