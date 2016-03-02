#include "iostream"
#include "string.h"
#include "fstream"

using namespace std;

static bool readConfigFile(const char *cfgfilepath, const string &key, string &value);
static bool printConfigFile();

//global variables
string img1Path,img2Path;
int c,except=0,number=0,found=0;

int main(int argc, char *argv[]){
    //load the configuration file
    char *filename=*++argv;
    --argc;
    ifstream in(filename);
    if(!in.is_open()){
        cerr<<"Error happened while opening the configuration file.\n";
        return -1;
    }
    //read key variables
    readConfigFile(filename,"img1Path",img1Path);
    readConfigFile(filename,"img2Path",img2Path);

    while(--argc>0 && (*++argv)[0]=='-')
        while(c=*++argv[0])
            switch (c){
            case 'p':
                printConfigFile();
                break;
            default:
                ;
            }

    return 0;
}

static bool readConfigFile(const char *cfgfilepath, const string &key, string &value){
    /*
     * parameter:
                 cfgfilepath - configuration file path
                         key - variable name in the configuration file
                       value - value that corresponds to the key
    */
    fstream cfgFile;
    cfgFile.open(cfgfilepath);
    if(!cfgFile.is_open()){
        cerr<<"Error happened while loading the configuration file!"<<endl;
        return -1;
    }
    char tmp[1000];
    while(!cfgFile.eof()){
        cfgFile.getline(tmp,1000);
        string line(tmp);
        size_t pos=line.find('=');
        if(pos==string::npos){
            cerr<<"Error happened while reading the configuration file!"<<endl;
            return -1;
        }
        string tmpKey=line.substr(0,pos);
        if(key==tmpKey){
            value=line.substr(pos+1);
            return 1;
        }
    }
    return -1;
}

static bool printConfigFile(){
    cout<<"Configuration File Content:"<<endl;
    cout<<"First Image Path:\t"<<img1Path<<endl;
    cout<<"Second Image Path:\t"<<img2Path<<endl;
    cout<<endl;
}
