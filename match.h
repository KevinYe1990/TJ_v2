#ifndef MATCH
#define MATCH

#include "utils.h"
#include "delaunay.h"


bool checkSize(const Mat& src,Rect range);

void refineMatches(const Mat& leftImg, const Mat& rightImg,const vector<Match>& src,vector<Match>& dst,
                     int windowSize=16,int torOfX=1,double mccThresh=0.8,bool resetY=false, int torOfY=1);

void nccMatch(const Mat &tpl, const Mat &src, Point2f &pt1, Point2f &pt2,
              double &maxCC, int& state,bool interpolation=true,double tolerance=1.0);

void matchUnderTerrainControl(const Mat& leftImg,const Mat& rightImg,const vector<Match>& matches4Ctrls,
                              vector<KeyPoint>& features,vector<Match>& matches,
                              int windowSize=16,int searchSize=24,int torOfEpipolar=2,double mccThresh=0.8);

Mat ransacTest(const vector<Match> &src, vector<Match> &dst,int method=CV_FM_RANSAC,
               double minDistance=1,double confidence=0.8);

bool surfaceFitting();

bool compMatches(const Match& m1, const Match& m2);

void updateTmpMatches(const vector<Match>& tmatches,string file=filename);

void filterOut(vector<Match>& matches,double from,double to,int mode=1);
void filterOut(vector<Match>& matches,double mcc);
#endif // MATCH

