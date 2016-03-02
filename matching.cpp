#include "matching.h"

//double nccMatch(const cv::Mat &tmp, const cv::Mat &src, cv::Point2f &pt1, cv::Point2f &pt2,
//                int& state,bool interpolation){
//    //create the result matrix
//    int result_cols=src.cols-tmp.cols+1;
//    int result_rows=src.rows-tmp.rows+1;
//    cv::Mat ccMat(result_rows,result_cols,CV_32F);
//    //Do the Matching and Normalize
//    cv::matchTemplate(tmp,src,ccMat,cv::TM_CCOEFF_NORMED);
//    //    cv::normalize(ccMat,ccMat,0,1,cv::NORM_MINMAX,-1,cv::Mat());
//    state=0;
//    //Localizing the best match with minMaxLoc
//    double minVal,maxVal;
//    cv::Point minLoc,maxLoc;
//    pt1.x=tmp.cols/2.0f;
//    pt1.y=tmp.rows/2.0f;
//    cv::minMaxLoc(ccMat,&minVal,&maxVal,&minLoc,&maxLoc,cv::Mat());
//    pt2.x=(float)maxLoc.x+int(tmp.cols/2)+.5f;//initial location
//    pt2.y=(float)maxLoc.y+int(tmp.rows/2)+.5f;
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
//            if(fabs(x_double-1.5)<.5 && fabs(y_double-1.5)<.5){
//                pt2.x=x_double+(float)maxLoc.x-1.5f+tmp.cols/2.0f;
//                pt2.y=y_double+(float)maxLoc.y-1.5f+tmp.rows/2.0f;
//                state=1;
//            }
//        }
//    }
//    return maxVal;
//}

double nccMatch(const cv::Mat &tmp, const cv::Mat &src, cv::Point2f &pt1, cv::Point2f &pt2,
                int& state,bool interpolation){
    //make sure that both the size of tmp and src are even
    assert(tmp.rows%2==0);
    assert(tmp.cols%2==0);
    assert(src.rows%2==0);
    assert(src.cols%2==0);
    //create the result matrix
    int result_cols=src.cols-tmp.cols+1;
    int result_rows=src.rows-tmp.rows+1;
    cv::Mat ccMat(result_rows,result_cols,CV_32F);
    //Do the Matching and Normalize
    cv::matchTemplate(tmp,src,ccMat,cv::TM_CCOEFF_NORMED);
    //    normalize(ccMat,ccMat,0,1,NORM_MINMAX,-1,Mat());
    state=0;
    //Localizing the best match with minMaxLoc
    double minVal,maxVal;
    cv::Point minLoc,maxLoc;
    pt1.x=tmp.cols/2.0f;
    pt1.y=tmp.rows/2.0f;
    cv::minMaxLoc(ccMat,&minVal,&maxVal,&minLoc,&maxLoc,cv::Mat());
    //initial location, and it will be covered if interpolation is performed
    pt2.x=(float)maxLoc.x+tmp.cols/2.0f;
    pt2.y=(float)maxLoc.y+tmp.rows/2.0f;
    //refine the location by using interpolation
    if(interpolation){
        if(checkSize(ccMat,cv::Rect(maxLoc.x-1,maxLoc.y-1,3,3))){
            cv::Mat patch=ccMat(cv::Rect(maxLoc.x-1,maxLoc.y-1,3,3));
            double x_double,y_double;
            cv::Point2d p1,p2,p3;
            std::vector<cv::Point2d> input;
            //fit a 2nd polynomial curve along the x direction
            p1.x=0.5;p2.x=1.5;p3.x=2.5;
            p1.y=patch.at<float>(1,0);input.push_back(p1);
            p2.y=patch.at<float>(1,1);input.push_back(p2);
            p3.y=patch.at<float>(1,2);input.push_back(p3);
            x_double=fit2ndPolynomial(input);
            //fit a 2nd polynomial curve along the y direction
            input.clear();
            p1.y=patch.at<float>(0,1);input.push_back(p1);
            p2.y=patch.at<float>(1,1);input.push_back(p2);
            p3.y=patch.at<float>(2,1);input.push_back(p3);
            y_double=fit2ndPolynomial(input);
            if(fabs(x_double-1.5)<.5 && fabs(y_double-1.5)<.5){
                pt2.x=pt2.x+x_double-1.5f;
                pt2.y=pt2.y+y_double-1.5f;
                state=1;
            }
        }
    }
    return maxVal;
}

void AreaBasedMatch(const cv::Mat& leftImage,const cv::Mat& rightImage,
                    const std::vector<cv::KeyPoint>& leftkpts,std::vector<Match>& matches,
                    int windowSize,int searchSize,int torOfEpipolar,double ccLimit,int Xdeviation){
    matches.clear();
    std::vector<cv::KeyPoint>::const_iterator iter;
    cv::Point2f pt,left_pt,right_pt;

    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);

    for(iter=leftkpts.begin();iter<leftkpts.end();++iter){
        //traverse all keypoints
        pt=(*iter).pt;
        //higher left corner of the template patch
        left_pt=pt-cv::Point2f(windowRadius,windowRadius);
        //check if the range is beyond the range of the left image
        cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
        if(checkSize(leftImage,range)){
            //crop the left image patch
            cv::Mat tmp=leftImage(range);
            //higher left corner of the search patch
            right_pt=pt+cv::Point2f(Xdeviation,0)-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
            //check if the range is beyond the range of the right image
            range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
            if((checkSize(rightImage,range))){
                //crop the right image patch
                cv::Mat src=rightImage(range);
                //normalized correlation coefficient(NCC)
                int state=0;
                cv::Point2f pt1,pt2;
                double mcc=nccMatch(tmp,src,pt1,pt2,state);
                if(state){
                    //suppress correspondences with low correlation coefficient
                    if (mcc>=ccLimit){
                        Match match;
                        match.p1=pt;
                        match.p2=right_pt+pt2;
                        match.cc=mcc;
                        match.windowSize=windowSize;
                        matches.push_back(match);
                    }
                }
            }
        }
    }
}


void matchUnderTriControl(const cv::Mat& leftImage,const cv::Mat& rightImage,
                          const std::vector<Match>& TriConstraint,
                          std::vector<cv::KeyPoint>& features,std::vector<Match>& matches,
                          int windowSize,int searchSize,int torOfEpipolar,double ccLimit,double Xtor){
    matches.clear();
    assert(Xtor>=0);
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);
    //generate Triangulations for both images
    std::vector<cv::DMatch> matches4Constrol;
    std::vector<cv::KeyPoint> Keypoints_left,Keypoints_right;
    MatchToDMatch(TriConstraint,matches4Constrol,Keypoints_left,Keypoints_right);
    Delaunay tri_left;
    tri_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    Delaunay tri_right;
    tri_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    tri_right.setTriangulation(tri_left.getTriangleList());
    //traversing the triangulation
    std::vector<cv::Vec6f> triangulation=tri_left.getTriangleCoordinates();
    std::vector<cv::Vec6f> triangulation_right=tri_right.getTriangleCoordinates();
    std::vector<cv::Vec6f>::iterator iter1=triangulation.begin();
    std::vector<cv::Vec6f>::iterator iter2=triangulation_right.begin();
    for(;iter1<triangulation.end();++iter1,++iter2){

        //find features within the triangle
        cv::Vec6f t1=(*iter1);
        cv::Vec6f t2=(*iter2);
        cv::Point2f p1(t1[0],t1[1]);
        cv::Point2f p2(t1[2],t1[3]);
        cv::Point2f p3(t1[4],t1[5]);
        std::vector<cv::Point2f> contour;
        contour.push_back(p1);
        contour.push_back(p2);
        contour.push_back(p3);

        std::vector<cv::Point2f> ptsInside;
        for( int i = 0; i < features.size(); ++i ){
            int state= cv::pointPolygonTest(contour, features[i].pt, false);
            if(state==1){
                ptsInside.push_back(features[i].pt);
            }
        }

        //traversing all features within the triangle
        std::vector<cv::Point2f>::iterator iter=ptsInside.begin();
        for(;iter<ptsInside.end();++iter){
            //Calculate the possible disparity of this point
            double disparity=prediction(t1,t2,(*iter));
            cv::Point2f pt,left_pt,right_pt;
            pt=(*iter);
            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
            //check if the range is beyond the range of the left image
            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
            if(checkSize(leftImage,range)){
                //crop the left image patch
                cv::Mat tmp=leftImage(range);
                //higher left corner of the search patch
                right_pt=pt-cv::Point2f(disparity,0)-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
                //check if the range is beyond the range of the right image
                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
                if((checkSize(rightImage,range))){
                    //crop the right image patch
                    cv::Mat src=rightImage(range);
                    //normalized correlation coefficient(NCC)
                    int state=0;
                    cv::Point2f pt1,pt2;
                    double mcc=nccMatch(tmp,src,pt1,pt2,state);

                    //state=1;

                    if(state){
                        //suppress correspondences with low correlation coefficient
                        if (mcc>=ccLimit){
                            Match match;
                            match.p1=pt;
                            match.p2=right_pt+pt2;
                            match.cc=mcc;
                            match.windowSize=windowSize;
                            if(((match.p1.x-match.p2.x)<(disparity+Xtor)) && ((match.p1.x-match.p2.x)>(disparity-Xtor))){
                                //disparity constraint
                                matches.push_back(match);
                            }
                        }
                    }
                }
            }
        }
    }
}

void matchUnderTriControl_OMP(const cv::Mat& leftImage,const cv::Mat& rightImage,const std::vector<Match>& controls,
                             std::vector<cv::KeyPoint>& features,std::vector<Match>& matches,
                             int windowSize,int searchSize,int torOfPy,double ccLimit,double torOfPx){
    //this function aims to derive matches on terrain area parallelly
    //pre-processing
    matches.clear();
    assert(torOfPx>=0 && torOfPy>=0 && windowSize>0 && searchSize>=windowSize && features.size()>0);
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);

    //generate Triangulations for both images
    std::vector<cv::DMatch> matches4Constrol;
    std::vector<cv::KeyPoint> Keypoints_left,Keypoints_right;
    MatchesDecompose(controls,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay tri_left;
    tri_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    Delaunay tri_right;
    tri_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    tri_right.setTriangulation(tri_left.getTriangleList());

    std::vector<cv::Vec6f> leftTIN=tri_left.getTriangleCoordinates();
    std::vector<cv::Vec6f> rightTIN=tri_right.getTriangleCoordinates();

    //initialize
    int *pointstate=new int[features.size()];
    for(int i=0;i<features.size();++i)
        pointstate[i]=-1;
    //traversing all triangles
    for(int i=0;i<leftTIN.size();++i){
        //get current triangle
        cv::Vec6f t1=leftTIN[i];
        cv::Point2f p1(t1[0],t1[1]);
        cv::Point2f p2(t1[2],t1[3]);
        cv::Point2f p3(t1[4],t1[5]);
        std::vector<cv::Point2f> contour;
        contour.push_back(p1);
        contour.push_back(p2);
        contour.push_back(p3);
        //traversing all points
#pragma omp parallel for
        for(int j=0;j<features.size();++j){
            int state=cv::pointPolygonTest(contour, features[j].pt, false);
            if(state==1)    pointstate[j]=i;
        }
    }

    int num_proc=omp_get_num_procs();
    std::vector<std::vector<Match> > match_in_all;
//    std::ofstream log;
//    log.open("C:/1124/L3/matchess.txt",std::ios_base::app);

    //traversing all feature points
#pragma omp parallel for schedule(static,1)
    for(int i=0;i<features.size();++i){
        //match in each proc
        if(pointstate[i]>=0){
            cv::Vec6f t1=leftTIN[pointstate[i]];
            cv::Vec6f t2=rightTIN[pointstate[i]];
            cv::Point2f pt,left_pt,right_pt;
            pt=features[i].pt;
            double disparity=prediction(t1,t2,pt);
            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
            //check if the range is beyond the range of the left image
            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
            if(checkSize(leftImage,range)){
                //crop the left image patch
                cv::Mat tmp=leftImage(range);
                //higher left corner of the search patch
                right_pt=pt-cv::Point2f(disparity,0)-cv::Point2f(searchRadius,torOfPy+windowRadius);
                //check if the range is beyond the range of the right image
                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfPy)*2);
                if((checkSize(rightImage,range))){
                    //crop the right image patch
                    cv::Mat src=rightImage(range);
                    //normalized correlation coefficient(NCC)
                    int state=0;
                    cv::Point2f pt1,pt2;
                    double mcc=nccMatch(tmp,src,pt1,pt2,state);
                    if(state){
                        //suppress correspondences with low correlation coefficient
                        if (mcc>=ccLimit){
                            Match match;
                            match.p1=pt;
                            match.p2=right_pt+pt2/*-cv::Point2f(0.5f,0.5f)*/;//#ISSUE#  why minus 0.5???//ISSUE SOLVED~
                            match.cc=mcc;
                            match.windowSize=windowSize;
                            if(fabs(match.p1.x-match.p2.x-disparity)<torOfPx){
                                //disparity constraint
#pragma omp critical
                                {
                                    matches.push_back(match);
//                                    log<<match.p1.x<<","<<match.p1.y<<","<<match.p2.x<<","<<match.p2.y<<","<<match.cc<<std::endl;
                                }
                            }
                        }
                    }
                }
            }
            //pushback the result of current proc to match_in_all
        }
    }
//    log.close();
    //merge result

}

void matchGlacierFeatures2(const cv::Mat& leftImage,const cv::Mat& rightImage,
                           const std::vector<Match>& triTerrain,const std::vector<Match>& triGlacier,
                           const std::vector<cv::Point2f>& features,const cv::Mat& angMap,std::vector<Match>& matches,
                           int windowSize,int searchSize,int torOfEpipolar,
                           double ccLimit,double Xtor,double Ytor){
    matches.clear();
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);

    //establish triangulation for both terrain and glacier
    std::vector<cv::DMatch> matches4Constrol;
    std::vector<cv::KeyPoint> Keypoints_left,Keypoints_right;

    //Terrain
    MatchToDMatch(triTerrain,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay terrainTRI_left,terrainTRI_right;
    terrainTRI_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    terrainTRI_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    terrainTRI_right.setTriangulation(terrainTRI_left.getTriangleList());

    matches4Constrol.clear();
    Keypoints_left.clear();
    Keypoints_right.clear();

    //Glacier
    MatchToDMatch(triGlacier,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay glacierTRI_left,glacierTRI_right;
    glacierTRI_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    glacierTRI_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    glacierTRI_right.setTriangulation(glacierTRI_left.getTriangleList());


    //traversing all features

    for(std::vector<cv::Point2f>::const_iterator iter_feature=features.begin();
        iter_feature<features.end();++iter_feature){

        cv::Point2f pt=(*iter_feature);
        cv::Point2f disparity,left_pt,right_pt;

        double x,y;
        x=pt.x;
        y=pt.y;

        double angle=angMap.at<float>(round(y*pow(2.0,LAYER_NUM-LAYER_ID-1)),round(x*pow(2.0,LAYER_NUM-LAYER_ID-1)));


        //find the corresponding Glacier Triangulation
        int idxGlacier=glacierTRI_left.findTriangle(pt);

        //find the corresponding Terrain Triangulation
        int idxTerrain=terrainTRI_left.findTriangle(pt);

        if(idxTerrain!=-1){

            cv::Vec6f tt1=terrainTRI_left.getTriangle(idxTerrain);
            cv::Vec6f tt2=terrainTRI_right.getTriangle(idxTerrain);

            disparity.x=prediction(tt1,tt2,pt);

            if(idxGlacier!=-1){
                tt1=glacierTRI_left.getTriangle(idxGlacier);
                tt2=glacierTRI_right.getTriangle(idxGlacier);

                disparity.y=prediction(tt1,tt2,pt,1);
            }else
                disparity.y=0;
            //            disTX=disparity.x;

            //Match
            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
            //check if the range is beyond the range of the left image
            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
            if(checkSize(leftImage,range)){
                //crop the left image patch
                cv::Mat tmp=leftImage(range);
                //higher left corner of the search patch
                right_pt=pt-disparity-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
                right_pt.x=(int)right_pt.x;
                //check if the range is beyond the range of the right image
                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
                if((checkSize(rightImage,range))){
                    //crop the right image patch
                    cv::Mat src=rightImage(range);
                    //normalized correlation coefficient(NCC)
                    int state=0;
                    cv::Point2f pt1,pt2;
                    double mcc=nccMatch(tmp,src,pt1,pt2,state);
                    if(state){
                        //suppress correspondences with low correlation coefficient
                        if (mcc>=ccLimit){
                            Match match;
                            match.p1=pt;
                            match.p2=right_pt+pt2;
                            match.windowSize=windowSize;
                            match.cc=mcc;

                            //Elimination
                            double py_g_4e=match.p1.y-match.p2.y;

                            bool con1=true;
                            bool con2=false;
                            bool con3=false;
                            bool con4=false;

                            //vertical parallax validation
                            if(match.p1.y-match.p2.y>disparity.y-Ytor && match.p1.y-match.p2.y<disparity.y+Ytor)
                                con2=true;


                            //total horizontal parallax validation
                            if(match.p1.x-match.p2.x>disparity.x-Xtor && match.p1.x-match.p2.x<disparity.x+Xtor)
                                con3=true;


                            //angle validation
                            if(py_g_4e*angle>=0)
                                con4=true;


                            if(con1 && con2 && con3 && con4){
                                //disparity constraint

                                double iceVelocity=(match.p1.y-match.p2.y)/sin(angle*PI/180)*33*6*pow(2.0,LAYER_NUM-LAYER_ID);
                                match.ang=angle;
                                match.Vel=iceVelocity;
                                matches.push_back(match);
                            }
                        }
                    }
                }
            }
        }
    }
}

void matchGlacierFeatures1(const cv::Mat& leftImage,const cv::Mat& rightImage,
                           const std::vector<Match>& triTerrain,const std::vector<cv::Point2f>& features,
                           const cv::Mat& angMap,std::vector<Match>& matches,
                           int windowSize,int searchSize,int torOfEpipolar,
                           double ccLimit,double Xtor,double Ytor1,double Ytor2){
    matches.clear();
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);

    //establish triangulation for both terrain
    std::vector<cv::DMatch> matches4Constrol;
    std::vector<cv::KeyPoint> Keypoints_left,Keypoints_right;

    //Terrain
    MatchToDMatch(triTerrain,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay terrainTRI_left,terrainTRI_right;
    terrainTRI_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    terrainTRI_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    terrainTRI_right.setTriangulation(terrainTRI_left.getTriangleList());


    //traversing all features
    for(std::vector<cv::Point2f>::const_iterator iter_feature=features.begin();
        iter_feature<features.end();++iter_feature){

        cv::Point2f pt=(*iter_feature);
        cv::Point2f disparity,left_pt,right_pt;

        double x,y;
        x=pt.x;
        y=pt.y;

        double angle=angMap.at<float>(round(y*pow(2.0,LAYER_NUM-LAYER_ID-1)),round(x*pow(2.0,LAYER_NUM-LAYER_ID-1)));

        //find the corrrespondimatchesng Terrain Triangulation
        int idxTerrain=terrainTRI_left.findTriangle(pt);

        if(idxTerrain!=-1){

            cv::Vec6f tt1=terrainTRI_left.getTriangle(idxTerrain);
            cv::Vec6f tt2=terrainTRI_right.getTriangle(idxTerrain);

            //            double disTX=0;

            disparity.x=prediction(tt1,tt2,pt);
            disparity.y=0;
            //            disTX=disparity.x;

            //Match
            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
            //check if the range is beyond the range of the left image
            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
            if(checkSize(leftImage,range)){
                //crop the left image patch
                cv::Mat tmp=leftImage(range);
                //higher left corner of the search patch
                right_pt=pt-disparity-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
                right_pt.x=(int)right_pt.x;
                //check if the range is beyond the range of the right image
                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
                if((checkSize(rightImage,range))){
                    //crop the right image patch
                    cv::Mat src=rightImage(range);
                    //normalized correlation coefficient(NCC)
                    int state=0;
                    cv::Point2f pt1,pt2;
                    double mcc=nccMatch(tmp,src,pt1,pt2,state);
                    if(state){
                        //suppress correspondences with low correlation coefficient
                        if (mcc>=ccLimit){
                            Match match;
                            match.p1=pt;
                            match.p2=right_pt+pt2;
                            match.windowSize=windowSize;
                            match.cc=mcc;

                            //Elimination
                            double py_g_4e=match.p1.y-match.p2.y;

                            bool con1=true;
                            bool con2=false;
                            bool con3=false;
                            bool con4=false;

                            //vertical parallax validation
                            if(match.p1.y-match.p2.y>Ytor1 && match.p1.y-match.p2.y<Ytor2)
                                con2=true;


                            //total horizontal parallax validation
                            if(match.p1.x-match.p2.x>disparity.x-Xtor && match.p1.x-match.p2.x<disparity.x+Xtor)
                                con3=true;


                            //angle validation
                            if(py_g_4e*angle>=0)
                                con4=true;


                            if(con1 && con2 && con3 && con4){
                                //disparity constraint

                                double iceVelocity=(match.p1.y-match.p2.y)/sin(angle*PI/180)*33*6*pow(2.0,LAYER_NUM-LAYER_ID);
                                match.ang=angle;
                                match.Vel=iceVelocity;
                                matches.push_back(match);
                            }
                        }
                    }
                }
            }
        }
    }
}

void matchGlacierFeatures2_OMP(const cv::Mat& leftImage,const cv::Mat& rightImage,
                          const std::vector<Match>& triTerrain,const std::vector<Match>& triGlacier,
                          const std::vector<cv::Point2f>& features,const cv::Mat& angMap,std::vector<Match>& matches,
                          int windowSize,int searchSize,int torOfEpipolar,
                          double ccLimit,double Xtor,double Ytor){
    matches.clear();
    int windowRadius=(windowSize/2);
    int searchRadius=(searchSize/2);

    //establish triangulation for both terrain and glacier
    std::vector<cv::DMatch> matches4Constrol;
    std::vector<cv::KeyPoint> Keypoints_left,Keypoints_right;

    //Terrain
    MatchToDMatch(triTerrain,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay terrainTRI_left,terrainTRI_right;
    terrainTRI_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    terrainTRI_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    terrainTRI_right.setTriangulation(terrainTRI_left.getTriangleList());

    matches4Constrol.clear();
    Keypoints_left.clear();
    Keypoints_right.clear();

    //Glacier
    MatchToDMatch(triGlacier,matches4Constrol,Keypoints_left,Keypoints_right);

    Delaunay glacierTRI_left,glacierTRI_right;
    glacierTRI_left.generateDelaunay(Keypoints_left,cv::Rect(0,0,leftImage.cols,leftImage.rows));
    glacierTRI_right.generateDelaunay(Keypoints_right,cv::Rect(0,0,rightImage.cols,rightImage.rows));
    glacierTRI_right.setTriangulation(glacierTRI_left.getTriangleList());


    //traversing all features
#pragma omp parallel for schedule(static,1)
    for(int k=0;k<features.size();++k){

        cv::Point2f pt=features[k];
        cv::Point2f disparity,left_pt,right_pt;

        double x,y;
        x=pt.x;
        y=pt.y;

        double angle=angMap.at<float>(round(y*pow(2.0,LAYER_NUM-LAYER_ID-1)),round(x*pow(2.0,LAYER_NUM-LAYER_ID-1)));


        //find the corresponding Glacier Triangulation
        int idxGlacier=glacierTRI_left.findTriangle(pt);

        //find the corresponding Terrain Triangulation
        int idxTerrain=terrainTRI_left.findTriangle(pt);

        if(idxTerrain!=-1){

            cv::Vec6f tt1=terrainTRI_left.getTriangle(idxTerrain);
            cv::Vec6f tt2=terrainTRI_right.getTriangle(idxTerrain);

            disparity.x=prediction(tt1,tt2,pt);

            if(idxGlacier!=-1){
                tt1=glacierTRI_left.getTriangle(idxGlacier);
                tt2=glacierTRI_right.getTriangle(idxGlacier);

                disparity.y=prediction(tt1,tt2,pt,1);
            }else
                disparity.y=0;
            //            disTX=disparity.x;

            //Match
            left_pt=pt-cv::Point2f(windowRadius,windowRadius);
            //check if the range is beyond the range of the left image
            cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
            if(checkSize(leftImage,range)){
                //crop the left image patch
                cv::Mat tmp=leftImage(range);
                //higher left corner of the search patch
                right_pt=pt-disparity-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
                right_pt.x=(int)right_pt.x;
                //check if the range is beyond the range of the right image
                range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(windowRadius+torOfEpipolar)*2);
                if((checkSize(rightImage,range))){
                    //crop the right image patch
                    cv::Mat src=rightImage(range);
                    //normalized correlation coefficient(NCC)
                    int state=0;
                    cv::Point2f pt1,pt2;
                    double mcc=nccMatch(tmp,src,pt1,pt2,state);
                    if(state){
                        //suppress correspondences with low correlation coefficient
                        if (mcc>=ccLimit){
                            Match match;
                            match.p1=pt;
                            match.p2=right_pt+pt2;
                            match.windowSize=windowSize;
                            match.cc=mcc;

                            //Elimination
                            double py_g_4e=match.p1.y-match.p2.y;

                            bool con1=true;
                            bool con2=false;
                            bool con3=false;
                            bool con4=false;

                            //vertical parallax validation
                            if(match.p1.y-match.p2.y>disparity.y-Ytor && match.p1.y-match.p2.y<disparity.y+Ytor)
                                con2=true;


                            //total horizontal parallax validation
                            if(match.p1.x-match.p2.x>disparity.x-Xtor && match.p1.x-match.p2.x<disparity.x+Xtor)
                                con3=true;


                            //angle validation
                            if(py_g_4e*angle>=0)
                                con4=true;


                            if(con1 && con2 && con3 && con4){
                                //disparity constraint
#pragma omp critical
                                {
                                    double iceVelocity=(match.p1.y-match.p2.y)/sin(angle*PI/180)*33*6*pow(2.0,LAYER_NUM-LAYER_ID);
                                    match.ang=angle;
                                    match.Vel=iceVelocity;
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



//Sub Functions
bool checkSize(const cv::Mat& src,cv::Rect range){
    bool con1=false,con2=false;
    if(range.x>=0 && range.y>=0)
        con1=true;
    if((range.x+range.width)<=src.cols && (range.y+range.height)<=src.rows)
        con2=true;
    return con1 && con2;
}

double fit2ndPolynomial(const std::vector<cv::Point2d> pts){
    assert(pts.size()==3);
    cv::Point2d p1,p2,p3;
    p1=pts[0];p2=pts[1];p3=pts[2];
    double a,b,/*c,*/l1,l2,l3,l4;
    l1=(p1.x-p2.x)*(p1.x+p2.x)*(p2.y-p3.y);
    l2=(p1.x-p2.x)*(p1.x+p2.x)*(p2.x-p3.x);
    l3=(p2.x-p3.x)*(p2.x+p3.x)*(p1.y-p2.y);
    l4=(p2.x-p3.x)*(p2.x+p3.x)*(p1.x-p2.x);
    b=(l3-l1)/(l4-l2);
    a=((p1.y-p2.y)-b*(p1.x-p2.x))/(pow(p1.x,2)-pow(p2.x,2));
    //    c=p1.y-b*p1.x-a*pow(p1.x,2);
    return -b/(2.0f*a);
}

double prediction(cv::Vec6f t1,cv::Vec6f t2,cv::Point2f pt,int flag){
    double disparity=0;

    cv::Mat A(6,6,CV_64FC1);
    cv::Mat L(6,1,CV_64FC1);

    for(int i=0;i<3;i++){
        double x1,y1,x2,y2;
        x1=t1[i*2];y1=t1[i*2+1];
        x2=t2[i*2];y2=t2[i*2+1];

        double row11[1][6]={1,x1,y1,0,0,0};
        cv::Mat row1(1,6,CV_64FC1,row11);
        row1.copyTo(A.row(2*i));

        double row22[1][6]={0,0,0,1,x1,y1};
        cv::Mat row2(1,6,CV_64FC1,row22);
        row2.copyTo(A.row(2*i+1));

        L.at<double>(2*i,0)=x2;
        L.at<double>(2*i+1,0)=y2;
    }

    cv::Mat X(6,1,CV_64FC1);
    X=A.inv()*L;
    if(flag==0)
        disparity=pt.x-(X.at<double>(0,0)+X.at<double>(1,0)*pt.x+X.at<double>(2,0)*pt.y);
        else
        disparity=pt.y-(X.at<double>(3,0)+X.at<double>(4,0)*pt.x+X.at<double>(5,0)*pt.y);

    return disparity;
}

double predictGlacierFeature(cv::Vec6f triTerrain1,cv::Vec6f triTerrain2,
                             cv::Vec6f triGlacier1,cv::Vec6f triGlacier2,
                             cv::Point2f pt,double angle,cv::Point2f& disparity){
    //Prediction
    double disTX,disGX,disGY;

    std::vector<cv::Point3f> glacierX,glacierY;
    cv::Point3f gpt1,gpt2,gpt3;
    gpt1.x=triGlacier1[0];gpt1.y=triGlacier1[1];gpt1.z=triGlacier1[1]-triGlacier2[1];
    gpt2.x=triGlacier1[2];gpt2.y=triGlacier1[3];gpt2.z=triGlacier1[3]-triGlacier2[3];
    gpt3.x=triGlacier1[4];gpt3.y=triGlacier1[5];gpt3.z=triGlacier1[5]-triGlacier2[5];
    glacierY.push_back(gpt1);
    glacierY.push_back(gpt2);
    glacierY.push_back(gpt3);

    disGY=fitSurface(glacierY,pt);

    if(disGY*angle<0)
        std::cerr<<"WARNING:Control Triangle is not consistent with the point direction.\n";


    cv::Point3f tpt1,tpt2,tpt3;
    tpt1.x=triTerrain1[0];tpt1.y=triTerrain1[1];tpt1.z=triTerrain1[0]-triTerrain2[0];
    tpt2.x=triTerrain1[2];tpt2.y=triTerrain1[3];tpt2.z=triTerrain1[2]-triTerrain2[2];
    tpt3.x=triTerrain1[4];tpt3.y=triTerrain1[5];tpt3.z=triTerrain1[4]-triTerrain2[4];
    glacierX.push_back(tpt1);
    glacierX.push_back(tpt2);
    glacierX.push_back(tpt3);

    disTX=fitSurface(glacierX,pt);

    if(fabs(angle)<90){
        disGX=fabs(disGY/tan(angle*PI/180.0))+disTX;//angle in degree
    }else{
        disGX=-fabs(disGY/tan(angle*PI/180.0))+disTX;//angle in degree
    }

    disparity.x=(float)disGX;
    disparity.y=(float)disGY;

    return disTX;//return the horizontal parallax caused by the topo
}

double fitSurface(const std::vector<cv::Point3f> gcps,const cv::Point2f pt){
    double disparity=0;
    double x1,y1,z1,x2,y2,z2,x3,y3,z3;
    x1=gcps[0].x;y1=gcps[0].y;z1=gcps[0].z;
    x2=gcps[1].x;y2=gcps[1].y;z2=gcps[1].z;
    x3=gcps[2].x;y3=gcps[2].y;z3=gcps[2].z;

    double A,B,C,D;
    A = y1*(z2-z3)+y2*(z3-z1)+y3*(z1-z2);
    B = z1*(x2-x3)+z2*(x3-x1)+z3*(x1-x2);
    C = x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2);
    D = -x1*(y2*z3-y3*z2)-x2*(y3*z1-y1*z3)-x3*(y1*z2-y2*z1);

    if(C!=0)
        disparity=-(A*pt.x+B*pt.y+D)/C;
    else
        disparity=z1;

    return disparity;
}

int passControls(const cv::Mat& leftImage,const cv::Mat& rightImage,
                   Match& match,int startSize,int endSize,int searchRange,int step,
                   double xtor,int torOfEpipolar,double ytor1,double ytor2,double ccLimit){
    //Match backup=match;
    //    int torOfEpipolar=(int)ytor;
    int feedback=0;
    cv::Point2f pt,left_pt,right_pt;
    //traversing all the sizes
    for(int wz=startSize;wz<endSize;wz+=step){
        int windowRadius=(wz/2);
        int searchRadius=windowRadius+searchRange;
        pt=match.p1;
        left_pt=pt-cv::Point2f(windowRadius,windowRadius);
        cv::Rect range(left_pt.x,left_pt.y,windowRadius*2,windowRadius*2);
        if(checkSize(leftImage,range)){
            cv::Mat tmp=leftImage(range);
            right_pt=match.p2-cv::Point2f(searchRadius,torOfEpipolar+windowRadius);
            range=cv::Rect(right_pt.x,right_pt.y,searchRadius*2,(torOfEpipolar+windowRadius)*2);
            if(checkSize(rightImage,range)){
                cv::Mat src=rightImage(range);
                int state=0;
                cv::Point2f pt1,pt2;
                double mcc=nccMatch(tmp,src,pt1,pt2,state);
                if(state){
                    if(mcc>=ccLimit){
                        //NOTE:xtor is the difference between the horizontal parallaxes before and after refining
                        //HOWEVER,ytor is ONLY the tolerant of vertical parallax
                        pt1=pt;
                        pt2=right_pt+pt2;
                        if((fabs(pt1.x-pt2.x-match.p1.x+match.p2.x)<xtor) &&
                                (((pt1.y-pt2.y)<ytor2) && ((pt1.y-pt2.y)>ytor1))){
                            match.p1=pt1;
                            match.p2=pt2;
                            match.cc=mcc;
                            match.windowSize=wz;
                            feedback=1;
                            break;
                        }
                    }
                }
            }
        }
    }
    return feedback;
}
