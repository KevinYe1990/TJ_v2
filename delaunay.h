#ifndef DELAUNAY
#define DELAUNAY

#include "common.h"

class Delaunay:Subdiv2D{

    typedef struct {
        int id;//triangle index
        Point2f vertices[3];
        double attribute[3];
        double area;
    } triangle;

public:
    Delaunay(Rect rect):Subdiv2D(rect){}
    Delaunay(const Mat& img):Subdiv2D(Rect(0,0,img.cols,img.rows)){}
    ~Delaunay(){}

    void generateDelaunay(const vector<Point2f>& pts);
    void generateDelaunay(const vector<KeyPoint>& kpts);
//    void generateDelaunay(const vector<Match>& matches);
//    void generateDelaunay(const vector<Point2f>& pts, const vector<double> attribute);
//    void getTriangulation();
    void getTriangulation(const vector<double>& attribute);
    void drawDelaunay(const Mat& src,Scalar delaunayColor=Scalar(0,0,255));

    //点在网内
    //根据三角网预测位置
private:
    vector<triangle> triangulation;
    double calTriArea(const Point2f pt1,const Point2f pt2,const Point2f pt3);

};

#endif // DELAUNAY

