#include "common.h"

bool exitwithErrors(const char *msg){
    cerr<<msg<<endl<<endl;
    return -1;
}

void trimString(string &str){
    int s=str.find_first_not_of(" ");
    int e=str.find_last_not_of(" ");
    str=str.substr(s,e-s+1);
    return;
}

bool str2bool(string s){
    trimString(s);
    transform(s.begin(),s.end(),s.begin(),::tolower);
    if(s=="true")
        return true;
    else
        return false;
}

int getTypeindex(const string t){
    string tstr=t;
    transform(tstr.begin(),tstr.end(),tstr.begin(),::tolower);
    if(tstr=="int")
        return 1;
    else if(tstr=="double")
        return 2;
    else if(tstr=="string")
        return 3;
    else if(tstr=="bool")
        return 4;
    else
        exitwithErrors("Cannot identify the unknown data type!");
}
