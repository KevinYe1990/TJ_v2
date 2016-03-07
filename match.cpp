#include "match.h"

//double nccMatch(const cv::Mat &tmp, const cv::Mat &src, cv::Point2f &pt1, cv::Point2f &pt2,
//                int& state,bool interpolation){
//    //make sure that both the size of tmp and src are even
//    assert(tmp.rows%2==0);
//    assert(tmp.cols%2==0);
//    assert(src.rows%2==0);
//    assert(src.cols%2==0);
//    //create the result matrix
//    int result_cols=src.cols-tmp.cols+1;
//    int result_rows=src.rows-tmp.rows+1;
//    cv::Mat ccMat(result_rows,result_cols,CV_32F);
//    //Do the Matching and Normalize
//    cv::matchTemplate(tmp,src,ccMat,cv::TM_CCOEFF_NORMED);
//    //    normalize(ccMat,ccMat,0,1,NORM_MINMAX,-1,Mat());
//    state=0;
//    //Localizing the best match with minMaxLoc
//    double minVal,maxVal;
//    cv::Point minLoc,maxLoc;
//    pt1.x=tmp.cols/2.0f;
//    pt1.y=tmp.rows/2.0f;
//    cv::minMaxLoc(ccMat,&minVal,&maxVal,&minLoc,&maxLoc,cv::Mat());
//    //initial location, and it will be covered if interpolation is performed
//    pt2.x=(float)maxLoc.x+tmp.cols/2.0f;
//    pt2.y=(float)maxLoc.y+tmp.rows/2.0f;
//    //refine the location by using interpolation
//    if(interpolation){
//        if(checkSize(ccMat,cv::Rect(maxLoc.x-1,maxLoc.y-1,3,3))){
//            cv::Mat patch=ccMat(cv::Rect(maxLoc.x-1,maxLoc.y-1,3,3));
//            double x_double,y_double;
//            cv::Point2d p1,p2,p3;
//            std::vector<cv::Point2d> input;
//            //fit a 2nd polynomial curve along the x direction
//            p1.x=0.5;p2.x=1.5;p3.x=2.5;
//            p1.y=patch.at<float>(1,0);input.push_back(p1);
//            p2.y=patch.at<float>(1,1);input.push_back(p2);
//            p3.y=patch.at<float>(1,2);input.push_back(p3);
//            x_double=fit2ndPolynomial(input);
//            //fit a 2nd polynomial curve along the y direction
//            input.clear();
//            p1.y=patch.at<float>(0,1);input.push_back(p1);
//            p2.y=patch.at<float>(1,1);input.push_back(p2);
//            p3.y=patch.at<float>(2,1);input.push_back(p3);
//            y_double=fit2ndPolynomial(input);
//            if(fabs(x_double-1.5)<1 && fabs(y_double-1.5)<1){
//                pt2.x=pt2.x+x_double-1.5f;
//                pt2.y=pt2.y+y_double-1.5f;
//                state=1;
//            }
//        }
//    }
//    return maxVal;
//}

//double fit2ndPolynomial(const std::vector<cv::Point2d> pts){
//    assert(pts.size()==3);
//    cv::Point2d p1,p2,p3;
//    p1=pts[0];p2=pts[1];p3=pts[2];
//    double a,b,/*c,*/l1,l2,l3,l4;
//    l1=(p1.x-p2.x)*(p1.x+p2.x)*(p2.y-p3.y);
//    l2=(p1.x-p2.x)*(p1.x+p2.x)*(p2.x-p3.x);
//    l3=(p2.x-p3.x)*(p2.x+p3.x)*(p1.y-p2.y);
//    l4=(p2.x-p3.x)*(p2.x+p3.x)*(p1.x-p2.x);
//    b=(l3-l1)/(l4-l2);
//    a=((p1.y-p2.y)-b*(p1.x-p2.x))/(pow(p1.x,2)-pow(p2.x,2));
//    //    c=p1.y-b*p1.x-a*pow(p1.x,2);
//    return -b/(2.0f*a);
//}

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



//bool fit2ndPoly(const Mat &cc, Point2d &pos){
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

void fit2ndPolynomial(const Mat &cc_Mat, double &x, double &y)
{
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
}
