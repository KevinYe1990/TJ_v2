#ifndef COMMON
#define COMMON

#include "iostream"
#include "string.h"
#include "fstream"
#include "sstream"
#include "algorithm"
#include "typeindex"
#include <unistd.h>

#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace std;
using namespace cv;


extern Mat img1,img2;
extern char *filename;
extern string directory;

bool exitwithErrors(const char *msg);
//int getTypeindex(const string t);
void lowerString(string &str);
void trimString(string &str);
bool str2bool(string s);

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);
bool readConfigFile(const char *cfgfilepath, const string &key, int &value);
bool readConfigFile(const char *cfgfilepath, const string &key, double &value);
bool readConfigFile(const char *cfgfilepath, const string &key, float &value);
bool readConfigFile(const char *cfgfilepath, const string &key, bool &value);

void showImage(Mat &img,string title="TEST",double scale=1);
void showKeypoints(const Mat img,const vector<KeyPoint> &kpts,double scale=1);

void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& kpts);

#endif // COMMON

