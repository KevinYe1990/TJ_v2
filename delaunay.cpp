#include "delaunay.h"

Delaunay::Delaunay(){}

Delaunay::~Delaunay(){}

int Delaunay::generateDelaunay(const vector<Point2f>& pts,Rect rect){
    //copy feature points to vertices
//    vertices=pts;
    triangulation.clear();

    //generate delaunay triangualtion
    Subdiv2D div(rect);
    for(int i = 0; i < pts.size(); ++i)
        div.insert(pts[i]);

    //store delaunay triangulation in variable 'triangulation'
    vector<cv::Vec6f> triangleList;
    div.getTriangleList(triangleList);
    int triIdx=0;
    for(vector<Vec6f>::iterator iter=triangleList.begin();iter<triangleList.end();++iter){
        //get the coordinates of three vertices of one triangle
        Vec6f t=(*iter);
        triangle tri;
        tri.tri_id=triIdx++;
        tri.pts[0]=Point2f(t[0],t[1]);
        tri.pts[1]=Point2f(t[2],t[3]);
        tri.pts[2]=Point2f(t[4],t[5]);
        triangulation.push_back(tri);
//        Point2f pts[3];
//        pts[0]=Point2f(t[0],t[1]);
//        pts[1]=Point2f(t[2],t[3]);
//        pts[2]=Point2f(t[4],t[5]);

//        //find indexes of three vertices
//        int pt_Idx[3];
//        pt_Idx[0]=-1;
//        pt_Idx[1]=-1;
//        pt_Idx[2]=-1;
//        bool outside=false;
//        for(int i=0;i<3;++i){
//            //            std::vector<cv::Point2f>::iterator iter_ver;
//            if(!outside){//beyond the image range}
//                for(int j=0;j<vertices.size();++j)
//                    if(pts[i]==vertices[j]){
//                        pt_Idx[i]=j;
//                        break;
//                    }
//                if(pt_Idx[i]==-1){
//                    outside=true;
//                    break;
//                }
//            }
//        }
//        if(!outside){
//            triangle tri;
//            tri.tri_id=triIdx++;
//            tri.p1_idx=pt_Idx[0];
//            tri.p2_idx=pt_Idx[1];
//            tri.p3_idx=pt_Idx[2];
//            triangulation.push_back(tri);
//        }
    }

    //return the number of the triangles
    return triangulation.size();
}

int Delaunay::generateDelaunay(const vector<KeyPoint>& kpts,Rect rect){
    vector<Point2f> pts;
    KeyPoint2Point2f(kpts,pts);
    return this->generateDelaunay(pts,rect);
}

int Delaunay::generateDelaunay(const vector<Match> &matches, Rect rect)
{
    triangulation.clear();
    triangulation2.clear();
    vector<Point2f> lpts,rpts;
    getPtsFromMatches(matches,lpts,rpts);

    //generate delaunay triangualtion
    Subdiv2D div(rect);
    for(int i = 0; i < lpts.size(); ++i)
        div.insert(pts[i]);

    //store delaunay triangulation in variable 'triangulation'
    vector<cv::Vec6f> triangleList;
    div.getTriangleList(triangleList);
    int triIdx=0;
    for(vector<Vec6f>::iterator iter=triangleList.begin();iter<triangleList.end();++iter){
        //get the coordinates of three vertices of one triangle
        Vec6f t=(*iter);
        triangle tri;
        tri.tri_id=triIdx++;
        tri.pts[0]=Point2f(t[0],t[1]);
        tri.pts[1]=Point2f(t[2],t[3]);
        tri.pts[2]=Point2f(t[4],t[5]);
        triangulation.push_back(tri);

    }

    //return the number of the triangles
    return triangulation.size();
}

void Delaunay::drawDelaunay(const Mat& src,Scalar delaunayColor){
    bool draw;
    Mat dst=src.clone();
    if(dst.type()==CV_8UC1){
        cvtColor(dst,dst,CV_GRAY2RGB);
    }
    for(size_t i = 0; i < triangulation.size(); ++i)
    {
        triangle tri = triangulation[i];
        // MY PIECE OF CODE
        draw=true;
        Point2f pts[3]=tri.pts;
        for(int i=0;i<3;i++){
            if((pts[i].x>src.cols-1)||(pts[i].y>src.rows-1)||(pts[i].x<0)||(pts[i].y<0))
                draw=false;
        }

        if (draw){
            cv::line(dst, pts[0], pts[1], delaunayColor, 1);
            cv::line(dst, pts[1], pts[2], delaunayColor, 1);
            cv::line(dst, pts[2], pts[0], delaunayColor, 1);
        }
    }
    showImage(dst);
}
