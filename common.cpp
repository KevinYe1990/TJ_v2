#include "common.h"

bool compKeyPoints(const KeyPoint& rhs, const KeyPoint& lhs){
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

void KeyPoint2Point2f(const vector<KeyPoint>& src, vector<Point2f>& dst){
    dst.clear();
    for (size_t i=0; i<src.size(); ++i)
        dst.push_back(src[i].pt);
}

void showMatches(const Mat& refImg,const Mat& schImg,const vector<Match> matches,double scale){
    assert(matches.size()>0);
    vector<DMatch> dmatches;
    vector<KeyPoint> leftkpts,rightkpts;
    Match2DMatch(matches,dmatches,leftkpts,rightkpts);

    Mat tmp;
    drawMatches(refImg,leftkpts,schImg,rightkpts,dmatches,tmp);
    resize(tmp,tmp,cv::Size(tmp.cols*scale,tmp.rows*scale));

    imshow("Matching Result.",tmp);
    waitKey(0);
}

void Match2DMatch(const vector<Match>& src,vector<DMatch>& dst,
                   vector<KeyPoint>& leftkpts,vector<KeyPoint>& rightkpts){
    assert(src.size()>0);
    std::vector<Match>::const_iterator iter=src.begin();
    dst.clear();
    leftkpts.clear();
    rightkpts.clear();
    int count=0;
    for(;iter<src.end();++iter){
        cv::KeyPoint kpt;
        kpt.pt=(*iter).p1;
        leftkpts.push_back(kpt);
        kpt.pt=(*iter).p2;
        rightkpts.push_back(kpt);
        cv::DMatch match;
        match.queryIdx=count;
        match.trainIdx=count;
        count+=1;
        dst.push_back(match);
    }
}

void Point2f2KeyPoint(const vector<Point2f>& src, vector<KeyPoint>& dst){
    dst.clear();
    for (size_t i=0; i<src.size();++i)
        dst.push_back(KeyPoint(src[i],1.0f));
}

bool exitwithErrors(const char *msg){
    cerr<<msg<<endl<<endl;
    return -1;
}

void trimString(string &str){
    int s=str.find_first_not_of(" ");
    int e=str.find_last_not_of(" ");
    if(e>s)
        str=str.substr(s,e-s+1);
    else
        str="";
}

bool str2bool(string s){
    trimString(s);
    lowerString(s);
    if(s=="true")
        return true;
    else
        return false;
}

void lowerString(string &str){
    transform(str.begin(),str.end(),str.begin(),::tolower);
}

void showImage(Mat &img,string title,double scale){
    Mat tmp=img;
    resize(tmp,tmp,Size(tmp.cols*scale,tmp.rows*scale));
    imshow(title,tmp);
    waitKey(0);
}

void printMatches(string filename,const vector<Match>& matches,int mode){
    ofstream out;
    switch(mode){
    case 0:
        out.open(filename,ios::out);
        break;
    case 1:
        out.open(filename,ios::app);
        break;
    default:
        exitwithErrors("Error occured while writing matching results!");
    }

    if(out.is_open()){
        vector<Match>::const_iterator iter;
        for(iter=matches.begin();iter!=matches.end();++iter){
            if(iter==matches.begin())
                out<<endl;
            if(iter==matches.end()-1)
                out<<(*iter).p1.x<<"\t"<<(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<(*iter).p2.y;
            else
                out<<(*iter).p1.x<<"\t"<<(*iter).p1.y<<"\t"<<(*iter).p2.x<<"\t"<<(*iter).p2.y<<endl;
        }
        out.close();
    }else
        exitwithErrors("Unable to open the output file!");

}

void showKeypoints(const Mat img,const vector<KeyPoint> &kpts,double scale){
    Mat tmp=img;
    if(kpts.size()!=0)
        drawKeypoints(img,kpts,tmp);
    cout<<kpts.size()<<" Keypoints were extracted..."<<endl;
    resize(tmp,tmp,Size(tmp.cols*scale,tmp.rows*scale));
    showImage(tmp);
}

void printKeypoints(std::string filename,const std::vector<cv::KeyPoint>& kpts){

    ofstream out;
    out.open(filename);
    int c=0;
    if(out.is_open()){
        vector<KeyPoint>::const_iterator iter;
        for(iter=kpts.begin();iter<kpts.end();++iter){
            out<<(*iter).pt.x<<"\t"<<(*iter).pt.y;
            if(++c!=kpts.size())
                out<<std::endl;
        }
        out.close();
    }else{
        exitwithErrors("Unable to open the output file!");
    }
    cout<<kpts.size()<<" Keypoints were written into the file "<<filename<<endl<<endl;
}

bool readConfigFile(const char *cfgfilepath, const string &key, string &value){
    /*
     * parameter:
                 cfgfilepath - configuration file path
                         key - variable name in the configuration file
                       value - value that corresponds to the key
    */
    fstream cfgFile;
    cfgFile.open(cfgfilepath);
    if(!cfgFile.is_open()){
        cerr<<"Error happened while loading the configuration file!"<<endl<<endl;
        return 0;
    }
    string line;
    while(!cfgFile.eof()){
        getline(cfgFile,line,'\n');
        trimString(line);
        if(line[0]=='#' || line.length()==0)
            //
            continue;
        else{
            size_t pos=line.find('=');
            if(pos==string::npos){
                cerr<<"Error happened while reading the configuration file!"<<endl;
                cfgFile.close();
                return 0;
            }
            string tmpKey=line.substr(0,pos);
            if(key==tmpKey){
                value=line.substr(pos+1);
                cfgFile.close();
                return 1;
            }
        }
    }
    cfgFile.close();
    return 0;
}
bool readConfigFile(const char *cfgfilepath, const string &key, int &value){
    string strtmp;
    bool b=readConfigFile(cfgfilepath,key,strtmp);
    if(b) value=atoi(strtmp.c_str());
    return b;
}
bool readConfigFile(const char *cfgfilepath, const string &key, double &value){
    string strtmp;
    bool b=readConfigFile(cfgfilepath,key,strtmp);
    if(b) value=atof(strtmp.c_str());
    return b;
}
bool readConfigFile(const char *cfgfilepath, const string &key, float &value){
    double tmp=value;
    bool b=readConfigFile(cfgfilepath,key,tmp);
    if(b) value=(float)tmp;
    return b;
}
bool readConfigFile(const char *cfgfilepath, const string &key, bool &value){
    string strtmp;
    bool b=readConfigFile(cfgfilepath,key,strtmp);
    if(b) value=str2bool(strtmp);
    return b;
}

void readMatches(const string filename, vector<Match> &matches, int withCC, int withWindowSize)
{
    matches.clear();
    ifstream in;
    in.open(filename);
    if(!in.is_open())
        exitwithErrors("Unable to read the file!");
    else{
        while(!in.eof()){
            Point2f pt1,pt2;
            Match match;
            if(withCC){
                double cc;
                in>>pt1.x>>pt1.y>>pt2.x>>pt2.y>>cc;
                match.corr=cc;
            }else
                in>>pt1.x>>pt1.y>>pt2.x>>pt2.y;

            match.p1=pt1;
            match.p2=pt2;
            if(withWindowSize){
                int ww;
                in>>ww;
                match.windowSize=ww;
            }
            matches.push_back(match);
        }
    }
    in.close();
}

void getPtsFromMatches(const vector<Match> &matches, vector<Point2f> &lpts, vector<Point2f> &rpts)
{
    assert(matches.size()>0);
    lpts.clear();
    rpts.clear();
    for(vector<Match>::const_iterator citer=matches.begin();citer<matches.end();++citer){
        Point2f lpt,rpt;
        lpt=(*citer).p1;
        lpts.push_back(lpt);
        rpt=(*citer).p2;
        rpts.push_back(rpt);
    }
}

void readKeyPoints(const string filename, vector<cv::KeyPoint>& kpts){
    ifstream in;
    in.open(filename);
    int numOfPoints=0;

    if(!in.is_open())
        exitwithErrors("Unable to open the keypoint file!");

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

void findIdentity(vector<KeyPoint> keypts, vector<Match> matches, vector<KeyPoint>& left){
    int n = keypts.size();
    int m = matches.size();
    vector<Point2f> lpts,rpts;
    vector<KeyPoint> r_pts;
    left.clear();
    rpts.clear();
    r_pts.clear();
    getPtsFromMatches(matches,rpts,lpts);

    KeyPoint::convert(rpts, r_pts);

    std::sort(keypts.begin(), keypts.end(),compKeyPoints);
    std::sort(r_pts.begin(), r_pts.end(), compKeyPoints);
    std::set_difference(
        keypts.begin(), keypts.end(),
        r_pts.begin(), r_pts.end(),
        std::back_inserter(left), compKeyPoints
        );

    std::printf("The total KeyPoint number is: %d.\n"
                "The number of the Matched ones is: %d.\n"
                "The number of left is: %d\n",
        n, m, left.size());
}

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

void fit2ndPolynomial(const Mat &cc_Mat, double &x, double &y){
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


//DEBUG
Mat genRandMat(int rows, int cols, int depth){
    assert(rows>0);
    assert(cols>0);
    Mat dst(rows,cols,depth);
    srand((unsigned)time(NULL));
    for(int i=0;i<rows;++i)
        for(int j=0;j<cols;++j){
            double r=rand()%256;
            switch(depth){
                case CV_32FC1:{
                    dst.at<float>(i,j)=(float)(r/256.0);
                    break;
                }
                case CV_64FC1:{
                    dst.at<double>(i,j)=r/256.0;
                    break;
                }
                default:{
                    dst.at<uchar>(i,j)=(int)r;
                    break;
                }
            }
        }
    return dst;
}

