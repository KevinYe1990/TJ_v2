#ifndef MATCH
#define MATCH

#include "common.h"

//old.match
void nccMatch(const cv::Mat &tpl, const cv::Mat &src, cv::Point2d &pt1, cv::Point2d &pt2,
              double &maxCC, int& state,bool interpolation=true);
//double fit2ndPolynomial(const std::vector<cv::Point2d> pts);
//double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag=0);
//double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt);

//new.match
void polyfit(const vector<double> xv,const vector<double> yv,vector<double> &coeff,int order=2);
bool fit2ndPolynomial(const Mat &cc_Mat, double &x, double &y);
bool checkSize(const cv::Mat& src,cv::Rect range);

#endif // MATCH

