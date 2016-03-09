#include "match.h"

void nccMatch(const Mat &tpl, const Mat &src, Point2d &pt1, Point2d &pt2,
                double &maxCC, int& state,bool interpolation){
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
    if(interpolation){
        if(checkSize(ccMat,Rect(maxLoc.x-1,maxLoc.y-1,3,3))){
            Mat patch=ccMat(Rect(maxLoc.x-1,maxLoc.y-1,3,3));
            double x,y;
            if(state=fit2ndPolynomial(patch,x,y)){
                pt2.x+=x-1.5;
                pt2.y+=y-1.5;
            }
        }
    }
}


//double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag){
//    double disparity=0;

//    cv::Mat A(6,6,CV_64FC1);
//    cv::Mat L(6,1,CV_64FC1);

//    for(int i=0;i<3;i++){
//        double x1,y1,x2,y2;
//        x1=t1[i*2];y1=t1[i*2+1];
//        x2=t2[i*2];y2=t2[i*2+1];

//        double row11[1][6]={1,x1,y1,0,0,0};
//        cv::Mat row1(1,6,CV_64FC1,row11);
//        row1.copyTo(A.row(2*i));

//        double row22[1][6]={0,0,0,1,x1,y1};
//        cv::Mat row2(1,6,CV_64FC1,row22);
//        row2.copyTo(A.row(2*i+1));

//        L.at<double>(2*i,0)=x2;
//        L.at<double>(2*i+1,0)=y2;
//    }

//    cv::Mat X(6,1,CV_64FC1);
//    X=A.inv()*L;
//    if(flag==0)
//        disparity=pt.x-(X.at<double>(0,0)+X.at<double>(1,0)*pt.x+X.at<double>(2,0)*pt.y);
//        else
//        disparity=pt.y-(X.at<double>(3,0)+X.at<double>(4,0)*pt.x+X.at<double>(5,0)*pt.y);

//    return disparity;
//}

////double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt){
//    double disparity=0;
//    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
//    x1=gcps[0].x;y1=gcps[0].y;z1=gcps[0].z;
//    x2=gcps[1].x;y2=gcps[1].y;z2=gcps[1].z;
//    x3=gcps[2].x;y3=gcps[2].y;z3=gcps[2].z;

//    double A,B,C,D;
//    A = y1*(z2-z3)+y2*(z3-z1)+y3*(z1-z2);
//    B = z1*(x2-x3)+z2*(x3-x1)+z3*(x1-x2);
//    C = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2);
//    D = -x1*(y2*z3-y3*z2)-x2*(y3*z1-y1*z3)-x3*(y1*z2-y2*z1);

//    if(C!=0)
//        disparity=-(A*pt.x+B*pt.y+D)/C;
//    else
//        disparity=z1;

//    return disparity;
//}



void polyfit(const vector<double> xv,const vector<double> yv,vector<double> &coeff,int order){
    MatrixXd A(xv.size(),order+1);
    VectorXd yv_mapped=VectorXd::Map(&yv.front(),yv.size());
    VectorXd result;

    assert(xv.size()==yv.size());
    assert(xv.size()>=order+1);

    //create matrix
    for (size_t i=0;i<xv.size();++i)
        for(size_t j=0;j<order+1;++j)
            A(i,j)=pow(xv.at(i),j);

    //solve for linear least squares fit
    result=A.householderQr().solve(yv_mapped);

    coeff.resize(order+1);
    for (size_t i=0;i<order+1;++i)
        coeff[i]=result[i];
}

bool fit2ndPolynomial(const Mat &cc_Mat, double &x, double &y){
    MatrixXd cc_eigen;
    cv2eigen(cc_Mat,cc_eigen);
    vector<double> vv={0.5,1.5,2.5};
    VectorXd v3d=cc_eigen.col(1);
    RowVectorXd r3d=cc_eigen.row(1);
    vector<double> xv(r3d.data(),r3d.data()+3);
    vector<double> yv(v3d.data(),v3d.data()+3);
    vector<double> coeff1,coeff2;
    polyfit(vv,xv,coeff1,2);
    polyfit(vv,yv,coeff2,2);
    x=-coeff1[1]/(2.0*coeff1[2]);
    y=-coeff2[1]/(2.0*coeff2[2]);

    double thresh=1.0;
    if((fabs(x-1.5)<=thresh) && (fabs(y-1.5)<=thresh))
        return 1;
    else
        return 0;
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
//    assert(Xtor>=0);
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);
    //generate Triangulations for both images
//    vector<DMatch> terrainCtrls;
//    vector<KeyPoint> left_kpts,right_kpts;
//    Match2DMatch(matches4Ctrls,terrainCtrls,left_kpts,right_kpts);
    Delaunay tri(leftImg);
//    tri.generateDelaunay(matches4Ctrls);

    //traversing the triangulation
//    int n=tri.getNumOfTRI();
//    for(int i=0;i<n;++i){
        //find features within the triangle

//    }

//
//        cv::Vec6f t1=(*iter1);
//        cv::Vec6f t2=(*iter2);
//        cv::Point2f p1(t1[0],t1[1]);
//        cv::Point2f p2(t1[2],t1[3]);
//        cv::Point2f p3(t1[4],t1[5]);
//        std::vector<cv::Point2f> contour;
//        contour.push_back(p1);
//        contour.push_back(p2);
//        contour.push_back(p3);

//        std::vector<cv::Point2f> ptsInside;
//        for( int i = 0; i < features.size(); ++i ){
//            int state= cv::pointPolygonTest(contour, features[i].pt, false);
//            if(state==1){
//                ptsInside.push_back(features[i].pt);
//            }
//        }

//        //traversing all features within the triangle
//        std::vector<cv::Point2f>::iterator iter=ptsInside.begin();
//        for(;iter<ptsInside.end();++iter){
//            //Calculate the possible disparity of this point
//            double disparity=prediction(t1,t2,(*iter));
//            cv::Point2f pt,left_pt,right_pt;
//            pt=(*iter);
//            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
//            //check if the range is beyond the range of the left image
//            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
//            if(checkSize(leftImage,range)){
//                //crop the left image patch
//                cv::Mat tmp=leftImage(range);
//                //higher left corner of the search patch
//                right_pt=pt-cv::Point2f(disparity,0)-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
//                //check if the range is beyond the range of the right image
//                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
//                if((checkSize(rightImage,range))){
//                    //crop the right image patch
//                    cv::Mat src=rightImage(range);
//                    //normalized correlation coefficient(NCC)
//                    int state=0;
//                    cv::Point2f pt1,pt2;
//                    double mcc=nccMatch(tmp,src,pt1,pt2,state);

//                    //state=1;

//                    if(state){
//                        //suppress correspondences with low correlation coefficient
//                        if (mcc>=ccLimit){
//                            Match match;
//                            match.p1=pt;
//                            match.p2=right_pt+pt2;
//                            match.cc=mcc;
//                            match.windowSize=windowSize;
//                            if(((match.p1.x-match.p2.x)<(disparity+Xtor)) && ((match.p1.x-match.p2.x)>(disparity-Xtor))){
//                                //disparity constraint
//                                matches.push_back(match);
//                            }
//                        }
//                    }
//                }
//            }
//        }
//    }
}


void Match2DMatch(const vector<Match> &src, vector<DMatch> &dst, vector<KeyPoint> &leftkpts, vector<KeyPoint> &rightkpts)
{
    assert(src.size()>0);
    dst.clear();
    leftkpts.clear();
    rightkpts.clear();
    int count=0;
    for(vector<Match>::const_iterator iter=src.begin();iter<src.end();++iter){
        KeyPoint kpt;
        kpt.pt=(*iter).p1;
        leftkpts.push_back(kpt);
        kpt.pt=(*iter).p2;
        rightkpts.push_back(kpt);
        DMatch match;
        match.queryIdx=count;
        match.trainIdx=count++;
        dst.push_back(match);
    }
}


