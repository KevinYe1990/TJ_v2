#ifndef DELAUNAY
#define DELAUNAY

#include "common.h"

class Delaunay:Subdiv2D{

    typedef struct {
        int id;//triangle index
        int pts_id[3];
        double area;
    } triangle;

    typedef struct {
        int id;
        Match mpt[3];
        double area;
    } dualTri;

public:
    Delaunay(Rect rect):Subdiv2D(rect){}
    Delaunay(const Mat& img):Subdiv2D(Rect(0,0,img.cols,img.rows)){}
    ~Delaunay(){}

    void generateDelaunay(const vector<Point2f>& pts);
    void generateDelaunay(const vector<KeyPoint>& kpts);
    void generateDelaunay(const vector<Match>& matches);
    void genTriangleList();
    void drawDelaunay(const Mat& src,Scalar delaunayColor=Scalar(0,0,255));

//    void getDTriList(vector<dualTri>& list);
    int getNumOfTRI();
    //点在网内
    //根据三角网预测位置
private:
//    vector<Point2f> vertices;
    vector<triangle> triList;
    vector<dualTri> dualList;
//    int numOfTRI;
};

#endif // DELAUNAY

