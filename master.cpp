#include "master.h"
#include "common.h"

extern Mat img1,img2;

bool extractFeatures(char *type){
    Ptr<FeatureDetector> detector;
    switch(type[0])
    {
        case '1':
        {//Good feature to track
            cout<<"Feature Extraction:\tGood feature to track..."<<endl<<endl;
            /*GoodFeaturesToTrackDetector( int maxCorners, double qualityLevel,
            double minDistance, int blockSize=3,bool useHarrisDetector=false, double k=0.04 );*/
//            detector=new GoodFeaturesToTrackDetector()
                    //new GoodFeaturesToTrackDetector(2000,0.001,5,6,false,0.04);
            break;
        }
        case '2':
        {//Sift feature to track
            cout<<"Feature Extraction:\tSift feature to track..."<<endl<<endl;
            break;
        }
        case '3':
        {//Grid feature to track
            cout<<"Feature Extraction:\tGrid feature to track..."<<endl<<endl;
            break;
        }
        default:
            exitwithErrors("unknown type for feature extraction!");
    }
}
