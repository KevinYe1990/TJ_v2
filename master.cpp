#include "master.h"
#include "common.h"

bool extractFeatures(char *type){
    Ptr<FeatureDetector> detector;
    vector<KeyPoint> keypoints;
    double scale=1;
    string ImageID="left",outPath=directory+"keypoints.txt";
    bool display=false;
    bool saveKpts=false;
    //read common key variables
    readConfigFile(filename,"ImageID",ImageID);
    readConfigFile(filename,"DisplayScale",scale);
    readConfigFile(filename,"DisplayKeypoints",display);
    readConfigFile(filename,"SaveKeypoints",saveKpts);
    readConfigFile(filename,"SaveKeypointsPath",outPath);
    lowerString(ImageID);
    //set detector
    switch(type[0])
    {
        case '1':
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
        case '2':
        {//Sift feature to track
            cout<<"Feature Extraction:\tSift feature to track..."<<endl<<endl;
//            SiftFeatureDetector()
            break;
        }
        case '3':
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
            showKeypoints(img2,keypoints,scale);
        else
            showKeypoints(img1,keypoints,scale);

    if(saveKpts){
        printKeypoints(outPath,keypoints);
    }
}
