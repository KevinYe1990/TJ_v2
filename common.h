#ifndef COMMON
#define COMMON

#include "iostream"
#include "string.h"
#include "fstream"
#include "sstream"
#include "algorithm"
#include "typeindex"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


extern Mat img1,img2;
extern char *filename;

bool exitwithErrors(const char *msg);
int getTypeindex(const string t);

void trimString(string &str);
bool str2bool(string s);

template <typename Type>
bool readConfigFile(const char *cfgfilepath, const string &key, Type &value){
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
        return -1;
    }
    char tmp[1000];
    string tmpstr;
    while(!cfgFile.eof()){
        cfgFile.getline(tmp,1000);
        if(tmp[0]=='#')
            break;
        else{
            string line(tmp);
            size_t pos=line.find('=');
            if(pos==string::npos){
                cerr<<"Error happened while reading the configuration file!"<<endl<<endl;
                return -1;
            }
            string tmpKey=line.substr(0,pos);
            if(key==tmpKey){
                tmpstr=line.substr(pos+1);
//                switch(getTypeindex(tmpstr)){
//                    case 1:
//                    {//int type
//                        value=atoi(tmpstr.c_str());
//                        break;
//                    }
//                    case 2:
//                    {//double type
//                        value=atof(tmpstr.c_str());
//                        break;
//                    }
//                    case 3:
//                    {//string type
                        value=tmpstr;
//                        break;
//                    }
//                    case 4:
//                    {//bool type
//                        value=str2bool(tmpstr);
//                        break;
//                    }
//                    default:
//                    exitwithErrors("Cannot identify the unknown data type!");//unknown data type
//                }
                return 1;
            }
        }
    }
    return -1;
}


#endif // COMMON

