#ifndef MATCH
#define MATCH

#include "common.h"

//old.match
double nccMatch(const cv::Mat &tmp, const cv::Mat &src, cv::Point2f &pt1, cv::Point2f &pt2,int& state,bool interpolation=true);
//double fit2ndPolynomial(const std::vector<cv::Point2d> pts);
//double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag=0);
//double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt);

//new.match
void polyfit(const vector<double> xv,const vector<double> yv,vector<double> &coeff,int order=2);
void fit2ndPolynomial(const Mat &cc,double &x,double &y);

#endif // MATCH

