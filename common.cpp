#include "common.h"

bool exitwithErrors(const char *msg){
    cerr<<msg<<endl<<endl;
    return -1;
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
        return -1;
    }
    char tmp[1000];
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
                value=line.substr(pos+1);
                return 1;
            }
        }
    }
    return -1;
}
