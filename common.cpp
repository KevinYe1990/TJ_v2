#include "common.h"
bool camp(const cv::KeyPoint& rhs, const cv::KeyPoint& lhs){
    if (rhs.pt.x < lhs.pt.x){
        return true;
    }
    else if (rhs.pt.x > lhs.pt.x)
        return false;
    else{
        if (rhs.pt.y < lhs.pt.y)
            return true;
        else
            return false;
    }
}

//Validation
void checkImage(const cv::Mat& src,std::string imgname){
    if(!src.data){
        std::cerr<<"Unable to open the image "+imgname+"!\n";
        std::exit(-1);
    }else{
        std::cout<<"Open the image "+imgname+" successfully!\n";
    }
}


//Display
void showKeypoints(const cv::Mat& src,std::vector<cv::KeyPoint>& pts,std::string title){
    cv::Mat dst;
    cv::drawKeypoints(src,pts,dst);
    //    resize(dst,dst,cv::Size(800,800));
    imshow(title,dst);
    cv::waitKey(0);
}

void showMatches(const cv::Mat& refImg,const cv::Mat& schImg,const std::vector<Match> matches,double scale){
    assert(matches.size()>0);
    std::vector<cv::DMatch> dmatches;
    std::vector<cv::KeyPoint> leftkpts,rightkpts;
    MatchToDMatch(matches,dmatches,leftkpts,rightkpts);

    cv::Mat tmp;
    cv::drawMatches(refImg,leftkpts,schImg,rightkpts,dmatches,tmp);
    if(scale!=0){
        cv::resize(tmp,tmp,cv::Size(tmp.cols*scale,tmp.rows*scale));
    }
    cv::imshow("Matching Result.",tmp);
    cv::waitKey(0);
}

//Input
void readKeyPoints(std::string filename,std::vector<cv::KeyPoint>& kpts){
    std::ifstream in;
    in.open(filename);
    int numOfPoints=0;

    if(!in.is_open()){
        std::cerr<<"Unable to open the keypoint file...\n";
        std::exit(-1);
    }

    kpts.clear();
    while(!in.eof()){
        cv::KeyPoint kpt;
        in>>kpt.pt.x>>kpt.pt.y;
        kpts.push_back(kpt);
        numOfPoints++;
    }
    in.close();
    printf("Input %d keypoints in all.\n",numOfPoints);
}

void readMatches(std::string filename,std::vector<Match>& matches,int withCC,int windowSize){
    matches.clear();
    std::ifstream in;
    in.open(filename);
    if(!in.is_open()){
        std::cerr<<"Unable to read the file.\n";
        exit(-1);
    }else{
        while(!in.eof()){
            cv::Point2f pt1,pt2;
            Match match;
            if(withCC){
                double cc;
                in>>pt1.x>>pt1.y>>pt2.x>>pt2.y>>cc;
                match.cc=cc;
            }else{
                in>>pt1.x>>pt1.y>>pt2.x>>pt2.y;
            }
            match.p1=pt1;
            match.p2=pt2;
            if(windowSize){
                int ww;
                in>>ww;
                match.windowSize=ww;
            }
            matches.push_back(match);
        }
    }
    in.close();
}

void readRecords(std::string filename,std::vector<Record>& records){
    std::vector<Match> matches;
    readMatches(filename,matches,true,true);
    records.clear();
    calRecords(matches,records);
}

//Output
void printMatches(std::string filename,const std::vector<Match>& matches){
    std::ofstream out;
    out.open(filename);
    if(out.is_open()){
        std::vector<Match>::const_iterator iter;
        for(iter=matches.begin();iter<matches.end();++iter){
            out<<(*iter).p1.x<<"\t"<<(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<(*iter).p2.y<<std::endl;
        }
        out.close();
    }else{
        std::cerr<<"Unable to open the output file...\n";
        exit(-1);
    }
}
void printMatches(std::string filename,const std::vector<Match>& matches,cv::Mat img){
    std::ofstream out;
    out.open(filename);
    double n=img.rows;
    out<<"F1"<<"\t"<<"F2"<<"\t"<<"F3"<<"\t"<<"F4"<<"\t"
      <<"mcc"<<"\t"<<"WS"<<"\t"<<"x1"<<"\t"<<"y1"<<"\t"<<"x2"<<"\t"<<"y2"<<"\t"<<"disX"<<"\t"<<"disY"<<std::endl;
    if(out.is_open()){
        std::vector<Match>::const_iterator iter;
        for(iter=matches.begin();iter<matches.end();++iter){
            out<<(*iter).p1.x<<"\t"<<(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<(*iter).p2.y
              <<"\t"<<(*iter).cc<<"\t"<<(*iter).windowSize<<"\t"
              <<(*iter).p1.x<<"\t"<<n-(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<n-(*iter).p2.y
             <<"\t"<<(*iter).p1.x-(*iter).p2.x<<"\t"<<(*iter).p1.y-(*iter).p2.y<<std::endl;
        }
        out.close();
    }else{
        std::cerr<<"Unable to open the output file...\n";
        exit(-1);
    }
}

void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& pts){

    std::ofstream out;
    out.open(filename);
    int c=0;
    if(out.is_open()){

        std::vector<cv::KeyPoint>::const_iterator iter;
        for(iter=pts.begin();iter<pts.end();++iter){
            out<<(*iter).pt.x<<"\t"<<(*iter).pt.y;
            c++;
            if(c!=pts.size())
                out<<std::endl;
        }
        out.close();
    }else{
        std::cerr<<"Unable to open the output file...\n";
        exit(-1);
    }
}

void printKeypoints_4_ARCGIS(std::string filename,const std::vector<cv::KeyPoint>& pts){
    std::ofstream out;
    out.open(filename);
    if(out.is_open()){
        out<<"f1\tf2\tx\ty\n";
        std::vector<cv::KeyPoint>::const_iterator iter;
        for(iter=pts.begin();iter<pts.end();++iter){
            double x,y;
            x=(*iter).pt.x;
            y=(*iter).pt.y;
            out<<x<<"\t"<<y<<"\t"<<x*pow(2.0,(double)(LAYER_NUM-LAYER_ID))-.5
              <<"\t"<<-y*pow(2.0,(double)(LAYER_NUM-LAYER_ID))+.5<<std::endl;
        }
        out.close();
    std::printf("%d features extracted...\n",pts.size());
    }else{
        std::cerr<<"Unable to open the output file...\n";
        exit(-1);
    }
}

void printMatches_4_ARCGIS(std::string filename,const std::vector<Match>& matches,int layer){
    std::ofstream out;
    out.open(filename);
    double scale=pow(2.0,(double)(LAYER_NUM-layer));
    out<<"F1"<<"\t"<<"F2"<<"\t"<<"F3"<<"\t"<<"F4"<<"\t"
      <<"mcc"<<"\t"<<"WS"<<"\t"<<"x1"<<"\t"<<"y1"<<"\t"<<"x2"<<"\t"<<"y2"<<"\t"<<"disX"<<"\t"<<"disY"<<std::endl;
    if(out.is_open()){
        std::vector<Match>::const_iterator iter;
        for(iter=matches.begin();iter<matches.end();++iter){
            out<<(*iter).p1.x<<"\t"<<(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<(*iter).p2.y
              <<"\t"<<(*iter).cc<<"\t"<<(*iter).windowSize<<"\t"
              <<(*iter).p1.x*scale-.5<<"\t"<<-(*iter).p1.y*scale+.5<<"\t"<<(*iter).p2.x*scale-.5<<"\t"<<-(*iter).p2.y*scale+.5
             <<"\t"<<((*iter).p1.x-(*iter).p2.x)*scale<<"\t"<<((*iter).p1.y-(*iter).p2.y)*scale<<std::endl;
        }
        out.close();
    }else{
        std::cerr<<"Unable to open the output file...\n";
        exit(-1);
    }
}

void printRecords(std::string filename,const std::vector<Match> matches,int layer){
    std::ofstream out(filename);
    int num=0;
    double scale=pow(2.0,(double)(LAYER_NUM-layer));
    if(!out.is_open()){
        std::cerr<<"ERROR: Unable to open the output file.\n";
        std::exit(-1);
    }else{
        out<<"f1\tf2\tf3\tf4\tmcc\tWS\tx1\ty1\tx2\ty2\tangle\tVelocity\n";
        std::vector<Match>::const_iterator iter=matches.begin();
        for(;iter<matches.end();++iter,++num){
            cv::Point2f p1,p2;
            Match match=(*iter);
            p1=match.p1;
            p2=match.p2;

            out<<p1.x*scale<<"\t"<<p1.y*scale<<"\t"<<p2.x*scale<<"\t"<<p2.y*scale<<"\t"<<match.cc<<"\t"<<match.windowSize<<"\t"<<
                 p1.x*scale-.5<<"\t"<<-p1.y*scale+.5<<"\t"<<p2.x*scale-.5<<"\t"<<-p2.y*scale+.5<<"\t"<<match.ang<<"\t"<<match.Vel<<std::endl;
        }
        out.close();
    }
    std::printf("%d records generated...\n",num);
}

void printRecords(std::string filename,const std::vector<Record>& records){
    std::ofstream out(filename);
    int num=0;
    if(!out.is_open()){
        std::cerr<<"ERROR: Unable to open the output file.\n";
        std::exit(-1);
    }else{
        out<<"f1\tf2\tf3\tf4\tmcc\tWS\tx1\ty1\tx2\ty2\tdisX\tdisY\n";
        std::vector<Record>::const_iterator iter=records.begin();
        for(;iter<records.end();++iter,++num){
            cv::Point2f p1,p2;
            Record r=(*iter);
            p1=r.match.p1;
            p2=r.match.p2;

            out<<p1.x<<"\t"<<p1.y<<"\t"<<p2.x<<"\t"<<p2.y<<"\t"<<r.match.cc<<"\t"<<r.match.windowSize<<"\t"<<
                 r.arc_x1<<"\t"<<r.arc_y1<<"\t"<<r.arc_x2<<"\t"<<r.arc_y2<<"\t"<<r.dx<<"\t"<<r.dy<<std::endl;
        }
        out.close();
    }
    std::printf("%d records generated...\n",num);
}

//Conversion
void MatchToDMatch(const std::vector<Match>& matches,std::vector<cv::DMatch>& out,
                   std::vector<cv::KeyPoint>& leftkpts,std::vector<cv::KeyPoint>& rightkpts){
    assert(matches.size()>0);
    std::vector<Match>::const_iterator iter=matches.begin();
    out.clear();
    leftkpts.clear();
    rightkpts.clear();
    int count=0;
    for(;iter<matches.end();++iter){
        cv::KeyPoint kpt;
        kpt.pt=(*iter).p1;
        leftkpts.push_back(kpt);
        kpt.pt=(*iter).p2;
        rightkpts.push_back(kpt);
        cv::DMatch match;
        match.queryIdx=count;
        match.trainIdx=count;
        count+=1;
        out.push_back(match);
    }
}

void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps) {
    ps.clear();
    for (unsigned int i=0; i<kps.size(); i++) ps.push_back(kps[i].pt);
}

void PointsToKeyPoints(const std::vector<cv::Point2f>& ps, std::vector<cv::KeyPoint>& kps) {
    kps.clear();
    for (unsigned int i=0; i<ps.size(); i++) kps.push_back(cv::KeyPoint(ps[i],1.0f));
}

//void findIdentity(const std::vector<cv::KeyPoint> keypts,const std::vector<Match> matches,std::vector<cv::KeyPoint>& left){
//    int n=keypts.size();
//    int m=matches.size();
//    left.clear();
//    for(int i=0;i<n;++i){
//        cv::Point2f pt1=(keypts[i]).pt;
//        bool found=false;
//        for(int j=0;j<m;++j){
//            cv::Point2f pt2=(matches[j]).p1;
//            double dist=sqrt(pow(pt1.x-pt2.x,2.0)+pow(pt1.y-pt2.y,2.0));
//            if(dist==0){
//                found=true;
//                break;
//            }
//        }
//        if(!found){
//            left.push_back(keypts[i]);
//        }
//    }
//    std::printf("The total KeyPoint number is: %d.\nThe number of the Matched ones is: %d.\nThe number of left is: %d\n",
//                n,m,left.size());
//}

/*void findIdentity(const std::vector<cv::KeyPoint> keypts,const std::vector<Match> matches,std::vector<Match>& left){
    int n=keypts.size();
    int m=matches.size();
    left.clear();
    for(int i=0;i<m;++i){
        cv::Point2f pt1=matches[i].p1;
        bool found=false;
        for(int j=0;j<n;++j){
            cv::Point2f pt2=keypts[j].pt;
            double dist=sqrt(pow(pt1.x-pt2.x,2.0)+pow(pt1.y-pt2.y,2.0));
            if(dist==0){
                found=true;
                break;
            }
        }
        if(!found){
            left.push_back(matches[i]);
        }
    }
    std::printf("The total KeyPoint number is: %d.\nThe number of the Matched ones is: %d.\nThe number of left is: %d\n",
                n,m,left.size());
}*/

//void findIdentity(const std::vector<cv::Point2f> pts,const std::vector<Match> matches,std::vector<cv::Point2f>& left){
//    std::vector<cv::KeyPoint> kpts,l;
//    PointsToKeyPoints(pts,kpts);
//    left.clear();
//    findIdentity(kpts,matches,l);
//    KeyPointsToPoints(l,left);
//}
void findIdentity(std::vector<cv::KeyPoint> keypts,  std::vector<Match> matches, std::vector<cv::KeyPoint>& left){
    int n = keypts.size();
    int m = matches.size();
    std::vector<cv::Point2f> rpts;
    std::vector<cv::KeyPoint> r_pts;
    left.clear();
    rpts.clear();
    r_pts.clear();
    for (int i = 0; i < m; ++i){
        cv::Point2f pt1;
        pt1 = (matches[i]).p1;
        rpts.push_back(pt1);
    }

    cv::KeyPoint::convert(rpts, r_pts);

    std::sort(keypts.begin(), keypts.end(),camp);
    std::sort(r_pts.begin(), r_pts.end(), camp);
    std::set_difference(
        keypts.begin(), keypts.end(),
        r_pts.begin(), r_pts.end(),
        std::back_inserter(left), camp
        );
    std::printf("The total KeyPoint number is: %d.\nThe number of the Matched ones is: %d.\nThe number of left is: %d\n",
        n, m, left.size());
}

//void findIdentity(const std::vector<Record>& r1,const std::vector<Record>& r2,std::vector<Record>& r){
//    //r2 is the correspondence set before, r1 is the correspondence set now
//    r.clear();
//    for(std::vector<Record>::const_iterator iter1=r1.begin();iter1<r1.end();++iter1){
//        cv::Point2f p1=(*iter1).match.p1;
//        bool found = false;
//        for(std::vector<Record>::const_iterator iter2=r2.begin();iter2<r2.end();++iter2){
//        cv::Point2f p2=(*iter2).match.p1;
//        if(p1.x-p2.x==0 && p1.y-p2.y==0){
//            found=true;
//        }
//        }
//        if(!found){
//            r.push_back(*iter1);
//        }
//    }
//    std::printf("The number of overlap is %d, and the number of left is %d.\n",r1.size()-r.size(),r.size());
//}


void generateDenseMap(const std::vector<Match>& matches,cv::Mat& out,int interval){
    cv::Size s=out.size();
    out=cv::Mat::zeros(s,CV_64F);
    std::vector<Match>::const_iterator iter;
    for(iter=matches.begin();iter<matches.end();++iter){
        Match match=*iter;
        out.at<double>(match.p1.y,match.p1.x)=1;
    }
    cv::imshow("test",out);
    cv::waitKey(0);

}


void MatchesDecompose(const std::vector<Match>& matches,std::vector<cv::DMatch>& out,std::vector<cv::KeyPoint>& leftkpts,std::vector<cv::KeyPoint>& rightkpts){
    assert(matches.size()>0);
    std::vector<Match>::const_iterator iter=matches.begin();
    out.clear();
    leftkpts.clear();
    rightkpts.clear();
    int count=0;
    for(;iter<matches.end();++iter){
        cv::KeyPoint kpt;
        kpt.pt=(*iter).p1;
        leftkpts.push_back(kpt);
        kpt.pt=(*iter).p2;
        rightkpts.push_back(kpt);
        cv::DMatch match;
        match.queryIdx=count;
        match.trainIdx=count;
        count+=1;
        out.push_back(match);
    }
}


void calRecords(const std::vector<Match>& matches,std::vector<Record>& records){
    records.clear();
    for(std::vector<Match>::const_iterator iter=matches.begin();iter<matches.end();++iter){
        Record r;
        r.match=*iter;
        r.dx=r.match.p1.x-r.match.p2.x;
        r.dy=r.match.p1.y-r.match.p2.y;
        r.arc_x1=r.match.p1.x-.5;
        r.arc_y1=-r.match.p1.y+.5;
        r.arc_x2=r.match.p2.x-.5;
        r.arc_y2=-r.match.p2.y+.5;
        records.push_back(r);
    }
}
