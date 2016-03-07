#ifndef DELAUNAY
#define DELAUNAY

#include "common.h"

struct triangle{
    int tri_id;//triangle index
//    int p1_idx;//indexes of three vertices in the variable 'vertices'
//    int p2_idx;
//    int p3_idx;
    Point2f pts[3];
};

class Delaunay{
public:
    Delaunay(/*const cv::Rect range*/);
    ~Delaunay();
    int generateDelaunay(const vector<Point2f>& pts,Rect rect);
    int generateDelaunay(const vector<KeyPoint>& kpts,Rect rect);
    int generateDelaunay(const vector<Match>& matches, Rect rect);
    void drawDelaunay(const Mat& src,Scalar delaunayColor=Scalar(0,0,255));
//    vector<triangle> getTriangleList();
//    void setTriangulation(const vector<triangle> tri);
//    vector<Vec6f> getTriangleCoordinates();
//    int findTriangle(const Point2f pt);
//    Vec6f getTriangle(int idx);

private:
//    vector<Point2f> vertices;
    vector<triangle> triangulation;
    vector<triangle> triangulation2;
};

#endif // DELAUNAY

