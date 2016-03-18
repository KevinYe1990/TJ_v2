#include "match.h"

void nccMatch(const Mat &tpl, const Mat &src, Point2f &pt1, Point2f &pt2,
                double &maxCC, int& state,bool interpolation,double tolerance){
    //make sure that both the size of tmp and src are even
    assert(tpl.rows%2==0);
    assert(tpl.cols%2==0);
    assert(src.rows%2==0);
    assert(src.cols%2==0);
    //create the result matrix
    int result_cols=src.cols-tpl.cols+1;
    int result_rows=src.rows-tpl.rows+1;
    Mat ccMat(result_rows,result_cols,CV_64FC1);
    //Do the Matching and Normalize
    matchTemplate(src,tpl,ccMat,TM_CCOEFF_NORMED);
    //    normalize(ccMat,ccMat,0,1,NORM_MINMAX,-1,Mat());
    //Localizing the best match with minMaxLoc
    double minVal;
    Point minLoc,maxLoc;
    pt1.x=tpl.cols/2.0;
    pt1.y=tpl.rows/2.0;
    minMaxLoc(ccMat,&minVal,&maxCC,&minLoc,&maxLoc,Mat());
    //initial location, and it will be covered if interpolation is performed
    pt2.x=(double)maxLoc.x+tpl.cols/2.0;
    pt2.y=(double)maxLoc.y+tpl.rows/2.0;
    //refine the location by using interpolation
    state=false;
    if(interpolation){
        if(checkSize(ccMat,Rect(maxLoc.x-1,maxLoc.y-1,3,3))){
            Mat patch=ccMat(Rect(maxLoc.x-1,maxLoc.y-1,3,3));
            double x,y;
            fit2ndPolynomial(patch,x,y);
            if(fabs(x-1.5)<=tolerance && fabs(y-1.5)<=tolerance){
                state=true;
                pt2.x+=x-1.5;
                pt2.y+=y-1.5;
            }
        }
    }
}

bool checkSize(const Mat& src,Rect range){
    bool con1=false,con2=false;
    if(range.x>=0 && range.y>=0)
        con1=true;
    if((range.x+range.width)<=src.cols && (range.y+range.height)<=src.rows)
        con2=true;
    return con1 && con2;
}

void matchUnderTerrainControl(const Mat& leftImg,const Mat& rightImg,const vector<Match>& matches4Ctrls,
                              vector<KeyPoint>& features,vector<Match>& matches /*output matches*/,
                              int windowSize,int searchSize,int torOfEpipolar,double mccThresh){
    //
    matches.clear();
    assert(windowSize%2==0);
    assert(searchSize%2==0);
    int shift=1;
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);
    Delaunay tri(leftImg);
    tri.generateDelaunay(matches4Ctrls);
    //traversing the triangulation
    int n=tri.getNumberOfTri();
    vector<bool> featureMask(features.size(),true);
    for(int i=0;i<n;++i){
        //find features within the triangle
        for(int j=0;j<features.size();++j){
            //traversing the features
            if(featureMask[j]){
                if(tri.iswithinTri(features[j].pt,i)){
                    //feature inside the triangle
                    Point2f feature=features[j].pt;
                    featureMask[j]=false;
                    double disparity=tri.interpolateAttr(feature,i);
                    Point2i PointOfLeftImg,PointOfRightImg;
                    PointOfLeftImg=Point2i(floor(feature.x-windowRadius),floor(feature.y-windowRadius));
                    //************************NOTE:***********************
                    //one pixel shift
                    PointOfRightImg=Point2i(floor(feature.x-disparity-searchRadius-shift),
                                            floor(feature.y-torOfEpipolar-windowRadius-shift));
                    //*****************************************************
                    //check if the range is beyond the range of the left image
                    Rect templateRange(PointOfLeftImg,Size(windowRadius*2,windowRadius*2));
                    //enlarge by one+one pixel
                    Rect searchRange(PointOfRightImg,Size((searchRadius+shift)*2,
                                                          (torOfEpipolar+windowRadius+shift)*2));
                    if(checkSize(leftImg,templateRange) && checkSize(rightImg,searchRange)){
                        //crop the patches
                        Mat templ=leftImg(templateRange);
                        Mat search=rightImg(searchRange);
                        //normalized correlation coefficient(NCC)
                        int state=0;
                        double mcc;
                        cv::Point2f pt1,pt2;
                        nccMatch(templ,search,pt1,pt2,mcc,state);
                        if(state){
                            //remove candidates with low correlation coefficient
                            if(mcc>=mccThresh){
                                Match match;
                                match.p1=feature;
                                match.p2=Point2f(PointOfRightImg.x,PointOfRightImg.y)+pt2;
                                match.corr=mcc;
                                match.windowSize=windowSize;
                                if(fabs(match.p1.y-match.p2.y)<=torOfEpipolar){
                                    //Y parallax constraint
                                    matches.push_back(match);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}





Mat ransacTest(const vector<Match> &src, vector<Match> &dst,int method,double minDistance,double confidence)
{
    vector<Match> backup=src;
    dst.clear();
    // Convert keypoints into Point2f
    vector<Point2f> queryPoints, trainPoints;
    Mat fundamentalMat;
    getPtsFromMatches(backup,queryPoints,trainPoints);
    // Compute F matrix using RANSAC
    vector<uchar> inliers(queryPoints.size(),0);
    if (queryPoints.size()>0&&trainPoints.size()>0)
    {
        fundamentalMat= findFundamentalMat(Mat(queryPoints),Mat(trainPoints), // matching points
                                           inliers,       // match status (inlier or outlier)
                                           method, // RANSAC method
                                           minDistance,  // distance to epipolar line
                                           confidence); // confidence probability


        // extract the surviving (inliers) matches
        vector<uchar>::const_iterator	itIn= inliers.begin();
        vector<Match>::const_iterator	itM= backup.begin();
        // for all matches
        for ( ;itIn!= inliers.end(); ++itIn, ++itM)
            if (*itIn) dst.push_back(*itM);

        // The F matrix will be recomputed with all accepted matches
        // Convert keypoints into Point2f for final F computation
        queryPoints.clear();
        trainPoints.clear();
        getPtsFromMatches(dst,queryPoints,trainPoints);
        // Compute 8-point F from all accepted matches
        if (queryPoints.size()>0&&trainPoints.size()>0)
            fundamentalMat= cv::findFundamentalMat(Mat(queryPoints),Mat(trainPoints), // matches
                                                   method);
    }

    return fundamentalMat;
}


void refineMatches(const Mat& leftImg, const Mat& rightImg,const vector<Match>& src,vector<Match>& dst,
                   int windowSize,int torOfX,double mccThresh, bool resetY, int torOfY){
    vector<Point2f> lpts,rpts;
    getPtsFromMatches(src,lpts,rpts);

    int windowRadius=windowSize/2;
    int shift=1;
    dst.clear();

    for(int i=0;i<lpts.size();++i){

        Point2i PointOfLeftImg=Point2i(floor(lpts[i].x-windowRadius),floor(lpts[i].y-windowRadius));
        Rect templateRange=Rect(PointOfLeftImg,Size(windowRadius*2,windowRadius*2));

        Point2i PointOfRightImg;
        if(resetY)
            PointOfRightImg=Point2i(floor(rpts[i].x-windowRadius-torOfX-shift),
                                    floor(lpts[i].y-windowRadius-torOfY-shift));
        else
            PointOfRightImg=Point2i(floor(rpts[i].x-windowRadius-torOfX-shift),
                                    floor(rpts[i].y-windowRadius-torOfY-shift));

        Rect searchRange=Rect(PointOfRightImg,Size((windowRadius+torOfX+shift)*2,(windowRadius+torOfY+shift)*2));

        if(checkSize(leftImg,templateRange) && checkSize(rightImg,searchRange)){
            Mat templ=leftImg(templateRange);
            Mat search=rightImg(searchRange);
            Point2f pt1,pt2;
            double mcc;
            int state=false;
            nccMatch(templ,search,pt1,pt2,mcc,state,true,1.0);
            if(mcc>=mccThresh)
                if(state){
                    Match match;
                    match.p1=lpts[i];
                    match.p2=Point2f(PointOfRightImg.x,PointOfRightImg.y)+pt2;
                    match.corr=mcc;
                    match.windowSize=windowSize;
                    dst.push_back(match);
                }
        }
    }
}


void filterOut(vector<Match>& matches, double from, double to, int mode)
{
    assert(from <= to);
    vector<Match>::iterator iter=matches.begin();
    while(iter!=matches.end()){
        double para;
        if(mode==0)
            para=iter->getParaX();
        else if(mode==1)
            para=iter->getParaY();

        if(para<from || para>to)
            //out of range
            iter=matches.erase(iter);
        else
            ++iter;
    }
}


void filterOut(vector<Match>& matches, double mcc)
{
    assert(matches.size()>0);
    vector<Match>::iterator iter=matches.begin();
    while(iter!=matches.end()){
    if(iter->corr<mcc)
        iter=matches.erase(iter);
    else
        ++iter;
    }
}
