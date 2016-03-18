#ifndef DELAUNAY
#define DELAUNAY

#include "utils.h"

class Delaunay:Subdiv2D{

    typedef struct {
        int id;
        Point2f pt;
        double attr;
    } vertex;

    typedef struct {
//        int id;//triangle index
//        Point2f vertices[3];
//        double attribute[3];
        vertex vtx[3];
        double area;
    } triangle;

public:
    Delaunay(Rect rect):Subdiv2D(rect){}
    Delaunay(const Mat& img):Subdiv2D(Rect(0,0,img.cols,img.rows)){}
    ~Delaunay(){}
    //generate the triangulation
    void generateDelaunay(const vector<Point2f>& pts,
                          const vector<double>& attribute=vector<double>{});
    void generateDelaunay(const vector<KeyPoint>& kpts,
                          const vector<double>& attribute=vector<double>{});
    void generateDelaunay(const vector<Match>& matches);
    //get the triangulation
    void getTriangulation(const vector<double>& attribute=vector<double>{});
    //draw the triangulation
    void drawDelaunay(const Mat& src,Scalar delaunayColor=Scalar(0,0,255));
    //get the number of triangles contained in the triangulation
    int getNumberOfTri(){return triangulation.size();}
    //check if the point is inside the certain triangle
    bool iswithinTri(const Point2f& pt,int tri_id);
    //interpolate the attribute
    double interpolateAttr(const Point2f& pt,int tri_id);

    //根据三角网预测位置
private:
    vector<triangle> triangulation;
    //calculate the area of the triangle
    double calTriArea(const Point2f pt1,const Point2f pt2,const Point2f pt3);
    //overload
    double calTriArea(const vertex* v);
    //compare the areas of triangles
    static bool compTri(const triangle& a, const triangle& b){return a.area>b.area;}
    //get the contour of the triangle
    void convert2Contour(const triangle& tri,vector<Point2f>& contour);
};

#endif // DELAUNAY

