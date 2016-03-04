#ifndef COMMON
#define COMMON

#include "iostream"
#include "string.h"
#include "fstream"
#include "sstream"
#include "algorithm"
#include "typeindex"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


extern Mat img1,img2;
extern char *filename;

bool exitwithErrors(const char *msg);
//int getTypeindex(const string t);

void trimString(string &str);
bool str2bool(string s);

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);
bool readConfigFile(const char *cfgfilepath, const string &key, int &value);
bool readConfigFile(const char *cfgfilepath, const string &key, double &value);
bool readConfigFile(const char *cfgfilepath, const string &key, bool &value);


#endif // COMMON

