#ifndef COMMON
#define COMMON

#include "iostream"
#include "string.h"
#include "fstream"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


bool exitwithErrors(const char *msg);

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);

#endif // COMMON

