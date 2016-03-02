#ifndef COMMON
#define COMMON

//Standard Library
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <omp.h>

//Open Library
#include "opencv2/opencv.hpp"

#include "delaunay.h"

//Data Structure
struct Match{
    cv::Point2f p1;
    cv::Point2f p2;
    int windowSize;
    double cc;
    double ang;
    double Vel;
};

struct Record{
    Match match;
    double arc_x1,arc_y1,arc_x2,arc_y2;
    double dx;
    double dy;
};

//Macro Definitions
#define PI acos(0)*2
#define LAYER_ID 5
#define LAYER_NUM 5

//Validation
void checkImage(const cv::Mat& src,std::string imgname="");


//Display
void showKeypoints(const cv::Mat& src,std::vector<cv::KeyPoint>& pts,std::string title="TEST");
void showMatches(const cv::Mat& refImg,const cv::Mat& schImg,const std::vector<Match> matches,double scale=0);

//Input
void readKeyPoints(std::string filename,std::vector<cv::KeyPoint>& kpts);
void readMatches(std::string filename,std::vector<Match>& matches,int withCC=false, int windowSize=false);
void readRecords(std::string filename,std::vector<Record>& records);

//Output
void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& pts);
void printKeypoints_4_ARCGIS(std::string filename,const std::vector<cv::KeyPoint>& pts);
void printMatches_4_ARCGIS(std::string filename,const std::vector<Match>& matches,int layer=LAYER_ID);
void generateDenseMap(const std::vector<Match>& matches,cv::Mat& out,int interval=100);
void printRecords(std::string filename,const std::vector<Match> matches,int layer=LAYER_ID);
void printRecords(std::string filename,const std::vector<Record>& records);
void printMatches(std::string filename,const std::vector<Match>& matches);
void printMatches(std::string filename,const std::vector<Match>& matches,cv::Mat img);
//Convension
bool camp(const cv::KeyPoint& rhs, const cv::KeyPoint& lhs);
void MatchToDMatch(const std::vector<Match>& matches,std::vector<cv::DMatch>& out,
                   std::vector<cv::KeyPoint>& leftkpts,std::vector<cv::KeyPoint>& rightkpts);
void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps);
void PointsToKeyPoints(const std::vector<cv::Point2f>& ps, std::vector<cv::KeyPoint>& kps);
void findIdentity(std::vector<cv::KeyPoint> keypts, std::vector<Match> matches, std::vector<cv::KeyPoint>& left);
//void findIdentity(const std::vector<cv::Point2f> pts,const std::vector<Match> matches,std::vector<cv::Point2f>& left);
//void findIdentity(const std::vector<cv::KeyPoint> keypts,const std::vector<Match> matches,std::vector<Match>& left);
//void findIdentity(const std::vector<Record>& r1,const std::vector<Record>& r2,std::vector<Record>& r);
void calRecords(const std::vector<Match>& matches,std::vector<Record>& records);

void MatchesDecompose(const std::vector<Match>& matches,std::vector<cv::DMatch>& out,std::vector<cv::KeyPoint>& leftkpts,std::vector<cv::KeyPoint>& rightkpts);
#endif // COMMON

