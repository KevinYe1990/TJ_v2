#include "core.h"
//#include "utils.h"
#include "mls_overlap.hpp"
bool extractFeatures(char *type){
    Ptr<FeatureDetector> detector;
    vector<KeyPoint> keypoints;
    string ImageID="left",outPath=directory+"keypoints.txt";
    bool display=false;
    bool saveKpts=false;
    //read common key variables
    readConfigFile(filename,"ImageID",ImageID);
    readConfigFile(filename,"DisplayKeypoints",display);
    readConfigFile(filename,"SaveKeypoints",saveKpts);
    readConfigFile(filename,"SaveKeypointsPath",outPath);
    lowerString(ImageID);
    //set detector
    switch(type[0])
    {
        case GoodFeature:
        {//Good feature to track
            cout<<"Feature Extraction:\tGood feature to track..."<<endl<<endl;
            /*GoodFeaturesToTrackDetector( int maxCorners, double qualityLevel,
            double minDistance, int blockSize=3,bool useHarrisDetector=false, double k=0.04 );*/
            //initialize the parameters
            int maxCorners=1e8,blockSize=5;
            double qualityLevel=0.01,minDistance=5,k=0.04;
            bool useHarrisDEtector=false;
            //set parameters
            readConfigFile(filename,"maxCorners",maxCorners);
            readConfigFile(filename,"blockSize",blockSize);
            readConfigFile(filename,"qualityLevel",qualityLevel);
            readConfigFile(filename,"minDistance",minDistance);
            readConfigFile(filename,"k",k);
            readConfigFile(filename,"useHarrisDetector",useHarrisDEtector);
            //initialize feature detector
            detector=new GoodFeaturesToTrackDetector(maxCorners,qualityLevel,minDistance,blockSize,useHarrisDEtector,k);
            //detect
            if(ImageID=="left")
                detector->detect(img1,keypoints);
            else if(ImageID=="right")
                detector->detect(img2,keypoints);
            else{
                detector->detect(img1,keypoints);
                cerr<<"Unknown option for the image_id, the left image was detected instead!"<<endl;
            }
            break;
        }
        case SiftFeature:
        {//Fast feature to track
            cout<<"Feature Extraction:\tFast feature to track..."<<endl<<endl;
            /*  FastFeatureDetector(int threshold=10,bool nonmaxSuppression=true)*/
            //initialize the parameters
            int threshold=1;
            bool nonmaxSuppression=true;
            //set parameters
            readConfigFile(filename,"threshold",threshold);
            readConfigFile(filename,"nonmaxSuppression",nonmaxSuppression);
            //initialize feature detector
            detector=new FastFeatureDetector(threshold, nonmaxSuppression);
            //detect
            if(ImageID=="left")
                detector->detect(img1,keypoints);
            else if(ImageID=="right")
                detector->detect(img2,keypoints);
            else{
                detector->detect(img1,keypoints);
                cerr<<"Unknown option for the image_id, the left image was detected instead!"<<endl;
            }
            break;
        }
        case GridFeature:
        {//Grid feature to track
            cout<<"Feature Extraction:\tGrid feature to track..."<<endl<<endl;
            /*DenseFeatureDetector(float initFeatureScale=1.f,int featureScaleLevels=1,float featureScaleMul=0.1f,
                   int initXyStep=6,int initImgBound=0,bool varyXyStepWithScale=true,bool varyImgBoundWithScale=false)*/
            float initFeatureScale=1.f,featureScaleMul=0.1f;
            int featureScaleLevels=1,initXyStep=6,initImgBound=0;
            bool varyXyStepWithScale=true,varyImgBoundWithScale=false;
            //set parameters
            readConfigFile(filename,"initFeatureScale",initFeatureScale);
            readConfigFile(filename,"featureScaleMul",featureScaleMul);
            readConfigFile(filename,"featureScaleLevels",featureScaleLevels);
            readConfigFile(filename,"initXyStep",initXyStep);
            readConfigFile(filename,"initImgBound",initImgBound);
            readConfigFile(filename,"varyXyStepWithScale",varyXyStepWithScale);
            readConfigFile(filename,"varyImgBoundWithScale",varyImgBoundWithScale);
            //initialize feature detector
            detector=new DenseFeatureDetector(initFeatureScale,featureScaleLevels,featureScaleMul,
                                              initXyStep,initImgBound,varyXyStepWithScale,varyImgBoundWithScale);
            //detect
            if(ImageID=="left")
                detector->detect(img1,keypoints);
            else if(ImageID=="right")
                detector->detect(img2,keypoints);
            else{
                detector->detect(img1,keypoints);
                cerr<<"Unknown option for the image_id, the left image was detected instead!"<<endl;
            }
            break;
        }
        default:
            exitwithErrors("unknown type for feature extraction!");
    }

    if(display)
        if(ImageID=="right")
            showKeypoints(img2,keypoints,imagescale);
        else
            showKeypoints(img1,keypoints,imagescale);

    if(saveKpts){
        printKeypoints(outPath,keypoints);
    }
}


void performMatching(char *type)
{
    int windowSize=16;
    double corrThreshold=0.6,paraYRangeFrom=-1.0,paraYRangeTo=1.0;
    bool displayMatches=false,saveMatchesAsTxt=false,saveMatchesAsShp;
    vector<Match> matches;
    string matchesToPrintPath,matchesShpToPrintPath,PATH_OF_STAGE1,PATH_OF_STAGE2;
    readConfigFile(filename,"windowSize",windowSize);
    readConfigFile(filename,"corrThreshold",corrThreshold);
    readConfigFile(filename,"paraYRangeFrom",paraYRangeFrom);
    readConfigFile(filename,"paraYRangeTo",paraYRangeTo);
    readConfigFile(filename,"displayMatches",displayMatches);
    readConfigFile(filename,"saveMatchesAsTxt",saveMatchesAsTxt);
    readConfigFile(filename,"saveMatchesAsShp",saveMatchesAsShp);
    readConfigFile(filename,"matchesShpToPrintPath",matchesShpToPrintPath);
    readConfigFile(filename,"matchesToPrintPath",matchesToPrintPath);
    readConfigFile(filename,"PATH_OF_STAGE1",PATH_OF_STAGE1);
    readConfigFile(filename,"PATH_OF_STAGE2",PATH_OF_STAGE2);

    switch(type[0]){
        case UnderTerrainControl:
    {
            string terrainCtrlsPath,featurePath;
            int searchSize=24,torOfEpipolar=1;
            readConfigFile(filename,"terrainCtrlsPath",terrainCtrlsPath);
            readConfigFile(filename,"featurePath",featurePath);
            readConfigFile(filename,"searchSize",searchSize);
            readConfigFile(filename,"torOfEpipolar",torOfEpipolar);

            vector<Match> terrain;
            vector<KeyPoint> features;
            readKeyPoints(featurePath,features);
            readMatches(terrainCtrlsPath,terrain);
            matchUnderTerrainControl(img1,img2,terrain,features,matches,
                                     windowSize,searchSize,torOfEpipolar,0);
            filterOut(matches,corrThreshold);
            cout<<"Update the stage 1 file..."<<endl;
            updateTmpMatches(matches,PATH_OF_STAGE1);
            filterOut(matches,paraYRangeFrom,paraYRangeTo,1);
            cout<<"Update the stage 2 file..."<<endl;
            updateTmpMatches(matches,PATH_OF_STAGE2);
            break;
        }
        case UnderGlacierControl:
    {
            break;
        }
        case RefineMatches:
    {
            string matchesToPassPath;
            int torlerantOfX=1,torlerantOfY=1;
            readConfigFile(filename,"matchesToPassPath",matchesToPassPath);
            readConfigFile(filename,"torlerantOfX",torlerantOfX);
            readConfigFile(filename,"torlerantOfY",torlerantOfY);
            readMatches(matchesToPassPath,matches);
            refineMatches(img1,img2,matches,matches,windowSize,
                          torlerantOfX,corrThreshold,false,torlerantOfY);
            filterOut(matches,paraYRangeFrom,paraYRangeTo,1);
            break;
        }
        default:{
            exitwithErrors("unknown type for matching!");
        }
    }

    if(displayMatches)  showMatches(img1,img2,matches,imagescale);

    if(saveMatchesAsTxt) printMatches(matchesToPrintPath,matches,0);
    if(saveMatchesAsShp) printShpfile(matchesShpToPrintPath,matches);
}


bool printConfigFile(){
    ifstream in(filename);
    if(!in.is_open()) exitwithErrors("Error occured while opening the configuration file!");
    while(!in.eof()){
        string line;
        getline(in,line,'\n');
        if(line[0]!='#')
            cout<<line<<endl;
    }
    in.close();
    return 0;
}


void surfaceFitting(char *type)
{
    string pointCloudInPath,pointCloudOutPath;
    int SFSearchRadius=25;
    readConfigFile(filename,"pointCloudInPath",pointCloudInPath);
    readConfigFile(filename,"pointCloudOutPath",pointCloudOutPath);
    readConfigFile(filename,"SFSearchRadius",SFSearchRadius);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ifstream in(pointCloudInPath);
    vector<int> state;
    int sum=0;//number of candidates
    if(in.is_open()){
        while(!in.eof()){
            pcl::PointXYZ pt;
            int sta;
            in>>pt.x>>pt.y>>pt.z>>sta;
            cloud->points.push_back(pt);
            state.push_back(sta);
            sum+=sta;
        }
    }else
        exitwithErrors("Errors occured while opening the point cloud file");
    in.close();
    cout<<state.size()<<" points inputed and "<<state.size()-sum<<" control points involved..."<<endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::MovingLeastSquaresOMP<pcl::PointXYZ,pcl::PointNormal> mls;
    //set parameters
    double searchRadius=SFSearchRadius;
    int order=std::atoi(type);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(order);
    mls.setSearchMethod(kdtree);
    mls.setSearchRadius(searchRadius);
    mls.setComputeNormals(true);
    mls.setSqrGaussParam(searchRadius*searchRadius);
    mls.setInputCloud(cloud);
    mls.setNumberOfThreads(16);
    //reconstruct
    mls.process(mls_points);

    //eliminate points with parallax difference over N times sigma
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointIndicesPtr crpd=mls.getCorrespondingIndices();
    for(int i=0;i!=crpd->indices.size();++i){
        pcl::PointXYZ point;
        point.x=cloud->points[crpd->indices[i]].x;
        point.y=cloud->points[crpd->indices[i]].y;
        point.z=cloud->points[crpd->indices[i]].z-mls_points.points[i].z;
        diff->points.push_back(point);
    }
    pcl::MLS_OMP<pcl::PointXYZ,pcl::PointXYZ> mls_;
    mls_.setSearchMethod(kdtree);
    mls_.setSearchRadius(searchRadius);
    mls_.setInputCloud(diff);
    mls_.setNumberOfThreads(16);

    //for all points
    for(int i=0;i!=diff->points.size();++i){
        //check if the point is a control point
        if(state[crpd->indices[i]]==0)
            continue;
        //find nearest points
        std::vector<int> indices_;
        std::vector<float> dist_;
        mls_.new_searchForNeighbors(i,indices_,dist_);
        //calculate mean and standard deviation of the local points
        std::vector<double> neighbors;
        double avg_neighbors,stdv_neighbors;
        for(int k=0;k!=indices_.size();++k)
            neighbors.push_back(diff->points[indices_[k]].z);
        get_avg_stdv(neighbors,avg_neighbors,stdv_neighbors);
        double lt=avg_neighbors+3.*stdv_neighbors;
        double gt=avg_neighbors-3.*stdv_neighbors;
        if((diff->points[i].z)>=gt && (diff->points[i].z)<=lt)
            state[crpd->indices[i]]=0;
    }

    //generate final result
    ofstream out(pointCloudOutPath);
    if(out.is_open()){
    for(int i=0;i!=cloud->points.size();++i)
        if(state[i]==0){
            out<<cloud->points[i].x<<"\t"<<cloud->points[i].y<<"\t"<<cloud->points[i].z;
            if(i!=cloud->points.size()-1)
                out<<endl;
        }
    }
    out.close();
}


void getIdentityMatches()
{
    string InMatchPath1,InMatchPath2/*,OutMatchPath*/;
    readConfigFile(filename,"InMatchPath1",InMatchPath1);
    readConfigFile(filename,"InMatchPath2",InMatchPath2);
//    readConfigFile(filename,"OutMatchPath",OutMatchPath);

    std::vector<Match> matches1,matches2,diff;
    readMatches(InMatchPath1,matches1);
    readMatches(InMatchPath2,matches2);
//    readMatches(OutMatchPath,diff);

    findIdentity(matches1,matches2,diff);
    size_t pos;
    pos=InMatchPath1.find_last_of('.');
    std::string comment="cp "+InMatchPath1+" "+InMatchPath1.substr(0,pos)+"_backup.txt";
    system(comment.c_str());
    printShortMatches(InMatchPath1,diff);
}
