#ifndef UTILS
#define UTILS

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <math.h>
#include <vector>
#include <stdlib.h>
//#include <typeindex>
#include <unistd.h>
#include <time.h>
#include <stdarg.h>
//OpenMP
#include <omp.h>
//Eigen 3.2.8
#include "Eigen/Dense"
//PCL 1.8
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/search/kdtree.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/surface/mls.h"
//OpenCV 2.4.9
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/eigen.hpp"
//GDAL
#include "gdal/gdal.h"
#include "gdal/ogr_api.h"
#include "gdal/ogrsf_frmts.h"
#include "gdal/ogr_spatialref.h"

using namespace std;
using namespace cv;
using namespace Eigen;

extern Mat img1,img2;
extern char *filename;
extern string directory;
extern double imagescale;

enum MATCH_TYPE{UnderTerrainControl='1',UnderGlacierControl='2',RefineMatches='3'};
enum FEATURE_TYPE{GoodFeature='1',SiftFeature='2',GridFeature='3'};

struct Match{
    Match(){}
    Match(Point2f p1,Point2f p2):p1(p1),p2(p2),windowSize(0),corr(0),angle(0),speed(0){}
    double getParaX() const {return (double)(p1.x-p2.x);}
    double getParaY() const {return (double)(p1.y-p2.y);}
    cv::Point2f p1;
    cv::Point2f p2;
    int windowSize;
    double corr;
    double angle;
    double speed;

    bool operator==(const Match& m) const {
        if(p1.x==m.p1.x && p1.y==m.p1.y)
            return true;
        return false;
    }
    bool operator<(const Match &m) const {
        if(p1.x<m.p1.x){
            return true;
        }
        else if(p1.x==m.p1.x && p1.y<m.p1.y){
            return true;
        }
        return false;
    }
    bool operator>(const Match &m) const {
        if(p1.x>m.p1.x){
            return true;
        }
        else if(p1.x==m.p1.x && p1.y>m.p1.y){
            return true;
        }
        return false;
    }
};

bool exitwithErrors(const char *msg);
void lowerString(string &str);
void trimString(string &str);
bool str2bool(string s);
void findIdentity(vector<KeyPoint> keypts, vector<Match> matches, vector<KeyPoint>& left);
void findIdentity(const vector<Match>& in_matches1,const vector<Match>& in_matches2,vector<Match>& diff);
bool compKeyPoints(const cv::KeyPoint& rhs, const cv::KeyPoint& lhs);
void polyfit(const vector<double> xv,const vector<double> yv,vector<double> &coeff,int order=2);
void fit2ndPolynomial(const Mat &cc_Mat, double &x, double &y);
void get_avg_stdv(const vector<double>& data,double& avg,double& stdv);
void KeyPoint2Point2f(const vector<KeyPoint>& src, vector<Point2f>& dst);
void Point2f2KeyPoint(const vector<Point2f>& src, vector<KeyPoint>& dst);
void Match2DMatch(const vector<Match>& src,vector<DMatch>& dst,
                   vector<KeyPoint>& leftkpts,vector<KeyPoint>& rightkpts);

bool readConfigFile(const char *cfgfilepath, const string &key, string &value);
bool readConfigFile(const char *cfgfilepath, const string &key, int &value);
bool readConfigFile(const char *cfgfilepath, const string &key, double &value);
bool readConfigFile(const char *cfgfilepath, const string &key, float &value);
bool readConfigFile(const char *cfgfilepath, const string &key, bool &value);
void readKeyPoints(const string filename,vector<cv::KeyPoint>& kpts);
void readMatches(const string filename,vector<Match>& matches,bool withTitle=false,bool withCC=false, bool withWindowSize=false,
                 bool withArcgisCoor=false,bool withParaXY=false);

void showImage(Mat &img,string title="TEST",double scale=1);
void showKeypoints(const Mat img,const vector<KeyPoint> &kpts,double scale=1);
void showMatches(const Mat& leftImg,const Mat& rightImg,const vector<Match> matches,double scale=1);

void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& kpts);
void printMatches(string filename,const vector<Match>& matches,int mode=0);
void printShortMatches(string filename,const vector<Match>& matches,int mode=0);

void getPtsFromMatches(const vector<Match>& matches,vector<Point2f>& lpts,vector<Point2f>& rpts);
void printShpfile(string shpname,const vector<Point2f>& pointSet,int EPSG=0);
void printShpfile(string shpname,const vector<KeyPoint>& pointSet,int EPSG=0);
void printShpfile(string shpname,const vector<Match>& matches,int EPSG=0);

//_DEBUG
Mat genRandMat(int rows,int cols,int depth=CV_8UC1);

#endif // UTILS

