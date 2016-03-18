#include "core.h"
//#include "utils.h"

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
    string matchesToPrintPath,matchesShpToPrintPath;
    readConfigFile(filename,"windowSize",windowSize);
    readConfigFile(filename,"corrThreshold",corrThreshold);
    readConfigFile(filename,"paraYRangeFrom",paraYRangeFrom);
    readConfigFile(filename,"paraYRangeTo",paraYRangeTo);
    readConfigFile(filename,"displayMatches",displayMatches);
    readConfigFile(filename,"saveMatchesAsTxt",saveMatchesAsTxt);
    readConfigFile(filename,"saveMatchesAsShp",saveMatchesAsShp);
    readConfigFile(filename,"matchesShpToPrintPath",matchesShpToPrintPath);
    readConfigFile(filename,"matchesToPrintPath",matchesToPrintPath);
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
            cout<<matches.size()<<endl;
            filterOut(matches,paraYRangeFrom,paraYRangeTo,1);
            cout<<matches.size()<<endl;

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
