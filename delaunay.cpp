#include "delaunay.h"

void Delaunay::drawDelaunay(const Mat& src,Scalar delaunayColor){
    bool draw;
    Mat dst=src.clone();
    if(dst.type()==CV_8UC1){
        cvtColor(dst,dst,CV_GRAY2RGB);
    }
    for(size_t i = 0; i < triangulation.size(); ++i)
    {
        triangle tri = triangulation[i];

        draw=true;
        Point2f pts[3];
        pts[0]=tri.vertices[0];
        pts[1]=tri.vertices[1];
        pts[2]=tri.vertices[2];

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

double Delaunay::calTriArea(const Point2f pt1, const Point2f pt2, const Point2f pt3)
{
//    vector< vector<Point2f>> contour;
    vector<Point2f> contour;
    contour.push_back(pt1);
    contour.push_back(pt2);
    contour.push_back(pt3);
//    contour.push_back(tri);
    return contourArea(contour);
}

void Delaunay::generateDelaunay(const vector<Point2f> &pts)
{
    this->insert(pts);
    getTriangulation();
}

void Delaunay::generateDelaunay(const vector<KeyPoint> &kpts){
    vector<Point2f> pts;
    KeyPoint2Point2f(kpts,pts);
    generateDelaunay(pts);
}

void Delaunay::getTriangulation(const vector<double> &attribute){
            triangulation.clear();
            int i, total = (int)(this->qedges.size()*4);
            vector<bool> edgemask(total, false);
            int idx=0;

            for( i = 4; i < total; i += 2 ){
                if( edgemask[i] )
                    continue;
                Point2f a, b, c;
                int edge = i;
                int A=edgeOrg(edge, &a);
                if (A<4) continue;
                edgeOrg(edge,&a);
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);
                int B=edgeOrg(edge, &b);
                if(B<4) continue;
                edgeOrg(edge, &b);
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);
                int C=edgeOrg(edge, &c);
                if(C<4) continue;
                edgeOrg(edge, &c);
                edgemask[edge] = true;

                triangle tri;
                tri.id=idx++;
                //                tri.pts_id[0]=A;
                //                tri.pts_id[1]=B;
                //                tri.pts_id[2]=C;
                tri.vertices[0]=a;
                tri.vertices[1]=b;
                tri.vertices[2]=c;
                tri.area=calTriArea(a,b,c);
                //???????????????
                tri.attribute[0]=attribute[A-4];
                tri.attribute[1]=attribute[B-4];
                tri.attribute[2]=attribute[C-4];

                triangulation.push_back(tri);
            }
}


//void Delaunay::generateDelaunay(const vector<Match> &matches){
//    vector<Point2f> left_pts,right_pts;
//    getPtsFromMatches(matches,left_pts,right_pts);
//    generateDelaunay(left_pts);

//    dualList.clear();

//    for(int i=0;i<triList.size();++i){
//        dualTri dtri;
//        dtri.id=i;
//        Match tmatches[3];
//        int idx=triList[i].pts_id[0];
//        tmatches[0]=Match(vtx[idx].pt,right_pts[idx-4]);//the first 4 vertices are the 4 corners
//        idx=triList[i].pts_id[1];
//        tmatches[1]=Match(vtx[idx].pt,right_pts[idx-4]);
//        idx=triList[i].pts_id[2];
//        tmatches[2]=Match(vtx[idx].pt,right_pts[idx-4]);


//    }
//}

//void Delaunay::generateDelaunay(const vector<Point2f> &pts, const vector<double> attribute){

//}

