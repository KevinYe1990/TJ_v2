#include "common.h"
#include "core.h"

//#define _debug
using namespace std;

//global variables and functions
string img1Path,img2Path,directory;
double imagescale=1.0;
char *filename;
Mat img1,img2;
int c/*,except=0,number=0,found=0*/;

static bool printConfigFile();
void getDirectory(string &str);

void help(){
    //guildline...
}

int main(int argc, char *argv[]){
    //print help
    help();

    //initilize
    getDirectory(directory);

    //load the configuration file
    filename=*++argv;
    --argc;
    ifstream in(filename);
    if(!in.is_open()){
        cerr<<"Error happened while opening the configuration file.\n";
        return -1;
    }

    //read key variables
    readConfigFile(filename,"img1Path",img1Path);
    img1=imread(img1Path);
    readConfigFile(filename,"img2Path",img2Path);
    img2=imread(img2Path);

    readConfigFile(filename,"DisplayScale",imagescale);

#ifdef _debug

#endif

    while(--argc>0 && (*++argv)[0]=='-')
        while(c=*++argv[0])
            switch (c){
            case 'm':
            {//match
                char *type=*++argv;
                --argc;
                if(strlen(type)==1)
                    performMatching(type);
                else
                    exitwithErrors("unknown type for feature extraction!");
                break;
            }
            case 'f':
            {//feature types: 1)Good Feature; 2)Sift Feature; 3)Grid Feature;
                char *type=*++argv;
                --argc;
                if(strlen(type)==1)
                    extractFeatures(type);
                else
                    exitwithErrors("unknown type for feature extraction!");
                break;
            }
            case 'p':
            {
                printConfigFile();
                break;
            }
            default:
                break;
            }

    return 0;
}

static bool printConfigFile(){
    cout<<"Configuration File Content:"<<endl;
    cout<<"First Image Path:\t"<<img1Path<<endl;
    cout<<"Second Image Path:\t"<<img2Path<<endl;
    cout<<endl;
    return 0;
}

void getDirectory(string &str){
int _max_length=1000;
char path[1000];
getcwd(path,_max_length);
str=string(path);
    str.append("/");
}
