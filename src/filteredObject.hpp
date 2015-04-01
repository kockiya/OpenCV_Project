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
	int x, y, adjusted_y, adjusted_x, ekernel_val, dkernel_val;
	bool can_track;
	double area;
	float radius;
	float scale;
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
		scale = 1;
		can_track = false;
	}
	
	void setName(string s){
		name = s;
	}
	
	
};

void changeScale(vector<filteredObject> &f, float newScale)
//Changes the pixel scale to provided scale.
{
	for(int i = 0; i < f.size(); i++)
	{
		f[i].scale = newScale;
	}
}


void saveSettings(vector<filteredObject> f, bool excludeSelected=false)
//If excludeSelected == true, the mask currently being calibrated will not be saved to the config.json.
{
	json file_export;
	json* entry = new json[f.size()];
	
	for(int i = 0; i < f.size()-(int)excludeSelected; i++)
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
		entry[i]["scale"] = to_string(f[i].scale);
		
		file_export[f[i].name] = entry[i];
	}
	
	ofstream out("config.json");
	
	out << file_export << endl;
	
	out.close();
	
	cout << "Settings saved to config.json" << endl;
}

bool restoreSettings(vector<filteredObject> &objects)
//Populates vector with config.json data
{

	//Check if config.json exists in working directory
	bool config_exists = false;
	ifstream fin("config.json");
	if(fin)
	{
		config_exists = true;
		fin.close();
	}
	
	//If it exists, populate vector elements with json data
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
			f.scale = stof(entry["scale"].as<string>());
		
			objects.push_back(f);

		}
		return true;
	}
	return false;
}



bool configureMasks(Mat &cameraFrame, Mat &hsvFrame, Mat &maskFrame, Mat &maskErode, vector<filteredObject> &objects, int z=-1)
{

	//Check if option parameter is valid.
	if(z >= objects.size())
		return false;
		
	//If z < 0, then draw all filtered objects. Otherwise only draw the selected one matching the index.
	//p starts at 0 if drawing all filtered objects; starts at z if drawing only z;
	//q is the size of the vector if drawwing all objects; is z+1 if drawing only z;
	int p = z < 0 ? 0 : z;
	int q = z < 0 ? objects.size() : z+1;
	
	

	for(int i = p; i < q; i++)
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
	return true;
	
}

bool updateFilteredObjectPosition(VideoCapture &cam, vector<filteredObject> &objects, int z = -1)
{
	if(!cam.isOpened())
		return false;
		
	//Check if option parameter is valid.
	if(z >= objects.size())
		return false;
	
	//If z < 0, then draw all filtered objects. Otherwise only draw the selected one matching the index.
	//p starts at 0 if drawing all filtered objects; starts at z if drawing only z;
	//q is the size of the vector if drawwing all objects; is z+1 if drawing only z;
	int p = z < 0 ? 0 : z;
	int q = z < 0 ? objects.size() : z+1;
	
	for(int i = p; i < q; i++)
	{
		findContours(objects[i].mask, objects[i].contours, objects[i].hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
				if(objects[i].hierarchy.size() > 0 && objects[i].contours.size() > 0)
				{
					for(int j = 0; j >= 0; j = objects[i].hierarchy[j][0])
					{
					//Credits for finding x, y, moments and area goes to youtube.com/khounslow as I do 
					//not understand what image moments are (computer vision related concept).
							objects[i].moment = moments((Mat)objects[i].contours[j]);
							objects[i].area = objects[i].moment.m00;
					
							if(objects[i].area > 100)
							{
								objects[i].x = objects[i].moment.m10/objects[i].area;
								objects[i].y = objects[i].moment.m01/objects[i].area;
								objects[i].adjusted_y = objects[i].mask.size().height - objects[i].moment.m01/objects[i].area;
								objects[i].adjusted_y *= objects[i].scale;
								objects[i].adjusted_x = objects[i].x * objects[i].scale;
								objects[i].can_track = true;
							}
							else
								objects[i].can_track = false;
					}
				}
				else
					objects[i].can_track = false;
	}
	
		
		return true;
}

bool drawFilteredObject(VideoCapture &cam, vector<filteredObject> &objects, Mat &cameraFrame, int z = -1)
{
	
	//Check if camera is opened
	if(!cam.isOpened())
		return false;
	
	//Check if option parameter is valid.
	if(z >= objects.size())
		return false;
	
	//If z < 0, then draw all filtered objects. Otherwise only draw the selected one matching the index.
	//p starts at 0 if drawing all filtered objects; starts at z if drawing only z;
	//q is the size of the vector if drawwing all objects; is z+1 if drawing only z;
	int p = z < 0 ? 0 : z;
	int q = z < 0 ? objects.size() : z+1;
	
	
	for(int i = p; i < q; i++)
	{
		if(objects[i].can_track)
		{
			//Credits for finding x, y, moments and area goes to youtube.com/khounslow as I do 
			//not understand what image moments are (computer vision related concept).
			objects[i].radius = sqrt(objects[i].area/3.14);
			int r = (int)objects[i].radius;
			const int xshift = 10, yshift = 10, center_size = 4;
			
			//Circle surrounding filtered object
			circle(cameraFrame, Point(objects[i].x, objects[i].y), r , Scalar(0, 255, 0), 2);
			
			//Point marking cente of filter object
			circle(cameraFrame, Point(objects[i].x, objects[i].y), center_size, Scalar(0, 255, 0), -1);
			
			r += 10; //Let r be the length of a new line segment that starts from center of circle and goes a bit out of the circle.
			line(cameraFrame, Point(objects[i].x, objects[i].y), Point(objects[i].x+xshift, objects[i].y+r), Scalar(0, 255, 0)); 
	
			//Text
			putText(cameraFrame, objects[i].name, Point(objects[i].x+xshift, objects[i].y+r), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
			putText(cameraFrame, "X: "+to_string(objects[i].adjusted_x) , Point(objects[i].x+xshift, objects[i].y+yshift+r), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
			putText(cameraFrame, "Y: "+to_string(objects[i].adjusted_y) , Point(objects[i].x+xshift, objects[i].y+2*yshift+r), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 255, 0));
		}
							
	}
		

		

	return true;

}

//Credits to stackoverflow Evan.Teran
vector<string> &split_str(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

vector<string> split_str(const string &s, char delim) {
    vector<string> elems;
    split_str(s, delim, elems);
    return elems;
}

#endif
