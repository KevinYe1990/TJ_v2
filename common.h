#ifndef COMMON
#define COMMON

#include "iostream"
#include "string.h"
#include "fstream"
#include "sstream"
#include "algorithm"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


bool exitwithErrors(const char *msg);

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);

void trimString(string &str);
bool str2bool(string s);

#endif // COMMON

