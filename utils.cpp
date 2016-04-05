#include "utils.h"

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

    cout<<matches.size()<<" correspondences were matched..."<<endl;

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
    if(matches.size()==0) return;
    ofstream out;
    switch(mode){
    case 0:
        out.open(filename,ios::out);
        out<<"F1\tF2\tF3\tF4\tMCC\tWindowSize\tPoint1_x\tPoint1_y\tPoint2_x\tPoint2_y\tParaX\tParaY"<<endl;
        break;
    case 1:
        out.open(filename,ios::app);
        out<<endl;
        break;
    default:
        exitwithErrors("Error occured while writing matching results!");
    }

    if(out.is_open()){
        vector<Match>::const_iterator iter;
        for(iter=matches.begin();iter!=matches.end();++iter){
            out<<iter->p1.x<<"\t"<<iter->p1.y<<"\t"<<iter->p2.x<<"\t"<<iter->p2.y<<"\t"<<iter->corr<<"\t"<<iter->windowSize<<"\t"
              <<iter->p1.x-.5<<"\t"<<-iter->p1.y+.5<<"\t"<<iter->p2.x-.5<<"\t"<<-iter->p2.y+.5<<"\t"
              <<iter->p1.x-iter->p2.x<<"\t"<<iter->p1.y-iter->p2.y;
            if(iter!=matches.end()-1) out<<endl;
        }
        out.close();
    }else
        exitwithErrors("Unable to open the output file!");
    cout<<matches.size()<<" correspondences were printed..."<<endl;
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

void readMatches(const string filename, vector<Match> &matches, bool withTitle,bool withCC, bool withWindowSize,bool withArcgisCoor,bool withParaXY)
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

            if(withTitle){
                string tmp;
                getline(in,tmp);
            }

            in>>pt1.x>>pt1.y>>pt2.x>>pt2.y;
            match.p1=pt1;
            match.p2=pt2;

            if(withCC){
                double cc;
                in>>cc;
                match.corr=cc;
            }

            if(withWindowSize){
                int ww;
                in>>ww;
                match.windowSize=ww;
            }
            if(withArcgisCoor){
                double tmpd;
                in>>tmpd>>tmpd>>tmpd>>tmpd;
            }
            if(withParaXY){
                double paraX,paraY;
                in>>paraX>>paraY;
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



void printShpfile(string shpname,const vector<Point2f>& pointSet,int EPSG)
{
    //（1）注册所有的文件格式驱动
    GDALAllRegister();
    OGRRegisterAll();

    //（2）得到shp文件的处理器
    OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");

    //（3）创建shp文件
    OGRDataSource* poDS = poDriver->CreateDataSource(shpname.c_str(), NULL );

    //（4）创建图层
    OGRSpatialReference* oSRS=new OGRSpatialReference();
    //    oSRS->SetWellKnownGeogCS("EPSG:3031");
    oSRS->importFromEPSG(EPSG);
    //    OGRLayer* poLayer= poDS->CreateLayer("Points", NULL, wkbLineString, NULL );
    OGRLayer* poLayer=poDS->CreateLayer("Points",oSRS,wkbPoint,NULL);

    //（5）创建字段
    // 字符串0
    //    OGRFieldDefn oField1("p1_x",OFTString);
    //    oField1.SetWidth(8);
    //    if( poLayer->CreateField( &oField1 ) != OGRERR_NONE )
    //        exitwithErrors("Creating Name field failed.\n" );

    // 整型
    OGRFieldDefn oField3("Point_ID",OFTInteger);
    if( poLayer->CreateField( &oField3 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    // 浮点数
    OGRFieldDefn oField1("Point_x",OFTReal);
    oField1.SetPrecision(3);
    if( poLayer->CreateField( &oField1 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField2("Point_y",OFTReal);
    oField2.SetPrecision(3);
    if( poLayer->CreateField( &oField2 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );



    for(int i=0;i!=pointSet.size();++i){
        //（6）创建几何和Feature
        OGRFeature *poFeature;
        poFeature =new OGRFeature( poLayer->GetLayerDefn() );

        poFeature->SetField( "Point_x", pointSet[i].x );
        poFeature->SetField( "Point_y", pointSet[i].y);
        poFeature->SetField( "Point_ID", i);
        //    OGRLineString *poLine =new OGRLineString();
        //    poLine->setNumPoints(2);
        //    poLine->setPoint(0,0.0,0.0, 0.0);
        //    poLine->setPoint(1,100.0,150.0, 0.0);

        OGRPoint *point=new OGRPoint();
        point->setX(pointSet[i].x-.5);
        point->setY(-pointSet[i].y+.5);

        poFeature->SetGeometry(point);
        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
            exitwithErrors("Failed to create feature in shapefile.");
        OGRFeature::DestroyFeature(poFeature);
    }
    //（7）资源清理
    OGRDataSource::DestroyDataSource( poDS );
    OGRCleanupAll();
}



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



void printShpfile(string shpname, const vector<KeyPoint> &pointSet, int EPSG)
{
    vector<Point2f> pts;
    KeyPoint2Point2f(pointSet,pts);
    printShpfile(shpname,pts,EPSG);
}


void printShpfile(string shpname, const vector<Match> &matches, int EPSG)
{
    //（1）注册所有的文件格式驱动
    GDALAllRegister();
    OGRRegisterAll();

    //（2）得到shp文件的处理器
    OGRSFDriver* poDriver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName("ESRI Shapefile");

    //（3）创建shp文件
    OGRDataSource* poDS = poDriver->CreateDataSource(shpname.c_str(), NULL );

    //（4）创建图层
    OGRSpatialReference* oSRS=new OGRSpatialReference();
    //    oSRS->SetWellKnownGeogCS("EPSG:3031");
    oSRS->importFromEPSG(EPSG);
    //    OGRLayer* poLayer= poDS->CreateLayer("Points", NULL, wkbLineString, NULL );
    OGRLayer* poLayer=poDS->CreateLayer("Points",oSRS,wkbPoint,NULL);

    //（5）创建字段
    // 字符串0
    //    OGRFieldDefn oField1("p1_x",OFTString);
    //    oField1.SetWidth(8);
    //    if( poLayer->CreateField( &oField1 ) != OGRERR_NONE )
    //        exitwithErrors("Creating Name field failed.\n" );

    // 整型
    OGRFieldDefn oField0("Match_ID",OFTInteger);
    if( poLayer->CreateField( &oField0 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    // 浮点数
    OGRFieldDefn oField1("Point1_x",OFTReal);
    oField1.SetPrecision(3);
    if( poLayer->CreateField( &oField1 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField2("Point1_y",OFTReal);
    oField2.SetPrecision(3);
    if( poLayer->CreateField( &oField2 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField3("Point2_x",OFTReal);
    oField3.SetPrecision(3);
    if( poLayer->CreateField( &oField3 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField4("Point2_y",OFTReal);
    oField4.SetPrecision(3);
    if( poLayer->CreateField( &oField4 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField5("MCC",OFTReal);
    oField5.SetPrecision(3);
    if( poLayer->CreateField( &oField5 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    OGRFieldDefn oField6("WindowSize",OFTInteger);
    if( poLayer->CreateField( &oField6 ) != OGRERR_NONE )
        exitwithErrors("Creating Name field failed.\n" );

    for(int i=0;i!=matches.size();++i){
        //（6）创建几何和Feature
        OGRFeature *poFeature;
        poFeature =new OGRFeature( poLayer->GetLayerDefn() );

        poFeature->SetField( "Match_ID", i);
        poFeature->SetField( "Point1_x", matches[i].p1.x );
        poFeature->SetField( "Point1_y", matches[i].p1.y);
        poFeature->SetField( "Point2_x", matches[i].p2.x );
        poFeature->SetField( "Point2_y", matches[i].p2.y);
        poFeature->SetField( "WindowSize", matches[i].windowSize);
        poFeature->SetField( "MCC", matches[i].corr);

        //    OGRLineString *poLine =new OGRLineString();
        //    poLine->setNumPoints(2);
        //    poLine->setPoint(0,0.0,0.0, 0.0);
        //    poLine->setPoint(1,100.0,150.0, 0.0);

        OGRPoint *point=new OGRPoint();
        point->setX(matches[i].p1.x-.5);
        point->setY(-matches[i].p1.y+.5);
        poFeature->SetGeometry(point);

        if( poLayer->CreateFeature( poFeature ) != OGRERR_NONE )
            exitwithErrors("Failed to create feature in shapefile.");
        OGRFeature::DestroyFeature(poFeature);
    }
    //（7）资源清理
    OGRDataSource::DestroyDataSource( poDS );
    OGRCleanupAll();
}
