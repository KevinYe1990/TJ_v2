#ifndef MATCHING
#define MATCHING

#include "common.h"


//Macro Definitions
#define YTOlERANT 5

//Matching Functions
double nccMatch(const cv::Mat &tmp, const cv::Mat &src, cv::Point2f &pt1, cv::Point2f &pt2,int& state,bool interpolation=true);

//NOTE: ONLY suitable for epipolar images
void AreaBasedMatch(const cv::Mat& leftImage,const cv::Mat& rightImage,
                        const std::vector<cv::KeyPoint>& leftkpts,std::vector<Match>& matches,
                        int windowSize=17,int searchSize=21,int torOfEpipolar=3,double ccLimit=0.8,int Xdeviation=0);

void matchUnderTriControl(const cv::Mat& leftImage,const cv::Mat& rightImage,const std::vector<Match>& TriConstraint,
                        std::vector<cv::KeyPoint>& features,std::vector<Match>& matches,
                        int windowSize=17,int searchSize=21,int torOfEpipolar=3,double ccLimit=0.8,double Xtor=1);

void matchUnderTriControl_OMP(const cv::Mat& leftImage,const cv::Mat& rightImage,const std::vector<Match>& controls,
                             std::vector<cv::KeyPoint>& features,std::vector<Match>& matches,
                             int windowSize,int searchSize,int torOfPy,double ccLimit,double torOfPx);

void matchGlacierFeatures1(const cv::Mat& leftImage,const cv::Mat& rightImage,
                          const std::vector<Match>& triTerrain,const std::vector<cv::Point2f>& features,
                           const cv::Mat& angMap,std::vector<Match>& matches,
                          int windowSize=17,int searchSize=21,int torOfEpipolar=1,
                          double ccLimit=0.8,double Xtor=0.5,double Ytor1=-.5,double Ytor2=0.5);

void matchGlacierFeatures2(const cv::Mat& leftImage,const cv::Mat& rightImage,
                          const std::vector<Match>& triTerrain,const std::vector<Match>& triGlacier,
                          const std::vector<cv::Point2f>& features,const cv::Mat& angMap,std::vector<Match>& matches,
                          int windowSize=17,int searchSize=21,int torOfEpipolar=1,
                          double ccLimit=0.8,double Xtor=0.5,double Ytor=0.5);

void matchGlacierFeatures2_OMP(const cv::Mat& leftImage,const cv::Mat& rightImage,
                          const std::vector<Match>& triTerrain,const std::vector<Match>& triGlacier,
                          const std::vector<cv::Point2f>& features,const cv::Mat& angMap,std::vector<Match>& matches,
                          int windowSize=17,int searchSize=21,int torOfEpipolar=1,
                          double ccLimit=0.8,double Xtor=0.5,double Ytor=0.5);

int passControls(const cv::Mat& leftImage,const cv::Mat& rightImage,
                   Match& match,int startSize,int endSize,int searchRange,int step,
                   double xtor,int torOfEpipolar,double ytor1,double ytor2,double ccLimit);

//Sub Functions
bool checkSize(const cv::Mat& src,cv::Rect range);

double fit2ndPolynomial(const std::vector<cv::Point2d> pts);

double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag=0);

double predictGlacierFeature(cv::Vec6f triTerrain1,cv::Vec6f triTerrain2,
                     cv::Vec6f triGlacier1,cv::Vec6f triGlacier2,
                     cv::Point2f pt,double angle,cv::Point2f& disparity);

double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt);

#endif // MATCHING

