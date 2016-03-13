#ifndef MATCH
#define MATCH

#include "utils.h"
#include "delaunay.h"

//old.match

//double fit2ndPolynomial(const std::vector<cv::Point2d> pts);
//double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag=0);
//double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt);

//new.match

bool checkSize(const Mat& src,Rect range);

void refineMatches(const Mat& leftImg, const Mat& rightImg,
                     const vector<Match>& src,vector<Match>& dst,
                     int windowSize=16,int torOfX=1,double mccThresh=0.8,
                     bool resetY=false, int torOfY=1);

void nccMatch(const Mat &tpl, const Mat &src, Point2f &pt1, Point2f &pt2,
              double &maxCC, int& state,bool interpolation=true,double tolerance=1.0);

void matchUnderTerrainControl(const Mat& leftImg,const Mat& rightImg,const vector<Match>& matches4Ctrls,
                              vector<KeyPoint>& features,vector<Match>& matches,
                              int windowSize=16,int searchSize=24,int torOfEpipolar=2,double mccThresh=0.8);

Mat ransacTest(const vector<Match> &src, vector<Match> &dst,int method=CV_FM_RANSAC,
               double minDistance=1,double confidence=0.8);
#endif // MATCH

