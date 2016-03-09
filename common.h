#ifndef COMMON
#define COMMON

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
//#include <typeindex>
#include <unistd.h>
#include <time.h>

//Eigen 3.2.8
#include "Eigen/QR"
#include "Eigen/Dense"
//OpenCV 2.4.9
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/eigen.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;

extern Mat img1,img2;
extern char *filename;
extern string directory;
extern double imagescale;

struct Match{
    Match(){}
    Match(Point2f p1,Point2f p2):p1(p1),p2(p2),windowSize(0),corr(0),angle(0),speed(0){}
    double getParaX(){return (double)(p1.x-p2.x);}
    double getParaY(){return (double)(p1.y-p2.y);}
    cv::Point2f p1;
    cv::Point2f p2;
    int windowSize;
    double corr;
    double angle;
    double speed;
};

bool exitwithErrors(const char *msg);
void lowerString(string &str);
void trimString(string &str);
bool str2bool(string s);
void KeyPoint2Point2f(const vector<KeyPoint>& src, vector<Point2f>& dst);
void Point2f2KeyPoint(const vector<Point2f>& src, vector<KeyPoint>& dst);

//void Matrix2Mat();

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);
bool readConfigFile(const char *cfgfilepath, const string &key, int &value);
bool readConfigFile(const char *cfgfilepath, const string &key, double &value);
bool readConfigFile(const char *cfgfilepath, const string &key, float &value);
bool readConfigFile(const char *cfgfilepath, const string &key, bool &value);

void showImage(Mat &img,string title="TEST",double scale=1);
void showKeypoints(const Mat img,const vector<KeyPoint> &kpts,double scale=1);
void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& kpts);

void readKeyPoints(const string filename,vector<cv::KeyPoint>& kpts);
void readMatches(const string filename,vector<Match>& matches,int withCC=false, int withWindowSize=false);
void getPtsFromMatches(const vector<Match>& matches,vector<Point2f>& lpts,vector<Point2f>& rpts);
void findIdentity(vector<KeyPoint> keypts, vector<Match> matches, vector<KeyPoint>& left);
bool compKeyPoints(const cv::KeyPoint& rhs, const cv::KeyPoint& lhs);


//_DEBUG
Mat genRandMat(int rows,int cols,int depth=CV_8UC1);

#endif // COMMON

