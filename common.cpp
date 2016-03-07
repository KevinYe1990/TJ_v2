#include "common.h"

void KeyPoint2Point2f(const vector<KeyPoint>& src, vector<Point2f>& dst){
    dst.clear();
    for (size_t i=0; i<src.size(); ++i)
        dst.push_back(src[i].pt);
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
    for(vector<Match>::const_iterator citer=matches.begin();citer=matches.end();++citer){
        Point2f lpt,rpt;
        lpt=(*citer).p1;
        lpts.push_back(lpt);
        rpt=(*citer).p2;
        rpts.push_back(rpt);
    }
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
