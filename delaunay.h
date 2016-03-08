#ifndef DELAUNAY
#define DELAUNAY

#include "common.h"

class Delaunay:Subdiv2D{
public:
    Delaunay(Rect rect):Subdiv2D(rect){}
    Delaunay(const Mat& img):Subdiv2D(Rect(0,0,img.cols,img.rows)){}
    ~Delaunay(){}

    void generateDelaunay(const vector<Point2f>& pts);
    void generateDelaunay(const vector<KeyPoint>& kpts);
    void generateDelaunay(const vector<Match>& matches);
    void getDTriList(vector<dualTri>& list);
    void genTriangleList();
    void drawDelaunay(const Mat& src,Scalar delaunayColor=Scalar(0,0,255));
private:
//    vector<Point2f> vertices;
    vector<triangle> triList;
    vector<dualTri> dualList;
};

#endif // DELAUNAY

