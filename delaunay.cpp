#include "delaunay.h"

void Delaunay::drawDelaunay(const Mat& src,Scalar delaunayColor){
    bool draw;
    Mat dst=src.clone();
    if(dst.type()==CV_8UC1){
        cvtColor(dst,dst,CV_GRAY2RGB);
    }
    for(size_t i = 0; i < triList.size(); ++i)
    {
        triangle tri = triList[i];

        draw=true;
        Point2f pts[3];
        pts[0]=vtx[tri.pts_id[0]].pt;
        pts[1]=vtx[tri.pts_id[1]].pt;
        pts[2]=vtx[tri.pts_id[2]].pt;

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

void Delaunay::generateDelaunay(const vector<Point2f> &pts)
{
    this->insert(pts);
    genTriangleList();
}

void Delaunay::generateDelaunay(const vector<KeyPoint> &kpts)
{
    vector<Point2f> pts;
    KeyPoint2Point2f(kpts,pts);
    generateDelaunay(pts);
}

void Delaunay::genTriangleList()
{
            triList.clear();
            int i, total = (int)(this->qedges.size()*4);
            vector<bool> edgemask(total, false);
            int idx=0;

            for( i = 4; i < total; i += 2 )
            {
                if( edgemask[i] )
                    continue;
                Point2f a, b, c;
                int edge = i;
                int A=edgeOrg(edge, &a);
                if (A<4) continue;
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);
                int B=edgeOrg(edge, &b);
                if(B<4) continue;
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);
                int C=edgeOrg(edge, &c);
                if(C<4) continue;
                edgemask[edge] = true;

                triangle tri;
                tri.id=idx++;
                tri.pts_id[0]=A;
                tri.pts_id[1]=B;
                tri.pts_id[2]=C;
                triList.push_back(tri);
            }
}

void Delaunay::generateDelaunay(const vector<Match> &matches)
{
    vector<Point2f> left_pts,right_pts;
    getPtsFromMatches(matches,left_pts,right_pts);
    generateDelaunay(left_pts);

    dualList.clear();

    for(int i=0;i<triList.size();++i){
        dualTri dtri;
        dtri.id=i;
        Match tmatches[3];
        int idx=triList[i].pts_id[0];
        tmatches[0]=Match(vtx[idx].pt,right_pts[idx-4]);//the first 4 vertices are the 4 corners
        idx=triList[i].pts_id[1];
        tmatches[1]=Match(vtx[idx].pt,right_pts[idx-4]);
        idx=triList[i].pts_id[2];
        tmatches[2]=Match(vtx[idx].pt,right_pts[idx-4]);

        memcpy(dtri.mpt,tmatches,3*sizeof(Match));

        dualList.push_back(dtri);
    }
}

void Delaunay::getDTriList(vector<dualTri> &list)
{
    list.clear();
    list.insert(list.end(),dualList.begin(),dualList.end());
}
