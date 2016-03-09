#ifndef MATCH
#define MATCH

#include "common.h"
#include "delaunay.h"

//old.match

//double fit2ndPolynomial(const std::vector<cv::Point2d> pts);
//double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag=0);
//double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt);

//new.match

bool checkSize(const Mat& src,Rect range);

void Match2DMatch(const vector<Match>& src,vector<cv::DMatch>& dst,
                   vector<KeyPoint>& leftkpts,vector<KeyPoint>& rightkpts);

void nccMatch(const Mat &tpl, const Mat &src, Point2d &pt1, Point2d &pt2,
              double &maxCC, int& state,bool interpolation=true);

void matchUnderTerrainControl(const Mat& leftImg,const Mat& rightImg,const vector<Match>& matches4Ctrls,
                              vector<KeyPoint>& features,vector<Match>& matches,
                              int windowSize=16,int searchSize=24,int torOfEpipolar=2,double mccThresh=0.8);


#endif // MATCH

