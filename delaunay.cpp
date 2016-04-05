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
        pts[0]=tri.vtx[0].pt;
        pts[1]=tri.vtx[1].pt;
        pts[2]=tri.vtx[2].pt;

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
    showImage(dst,"Delaunay",imagescale);
}


bool Delaunay::iswithinTri(const Point2f &pt, int tri_id)
{
    triangle tri=triangulation[tri_id];
    vector<Point2f> contour;
    convert2Contour(tri,contour);
    return pointPolygonTest(contour,pt,false)>0;
}


double Delaunay::interpolateAttr(const Point2f &pt, int tri_id){
    MatrixXd A(3,3);
    Vector3d yv_mapped(-1,-1,-1);
    VectorXd coeff;
    triangle tri=triangulation[tri_id];

    //create matrix
    for (size_t i=0;i<3;++i){
        A(i,0)=tri.vtx[i].pt.x;
        A(i,1)=tri.vtx[i].pt.y;
        A(i,2)=tri.vtx[i].attr;
    }

    //solve for linear least squares fit
    ColPivHouseholderQR<MatrixXd> qr_decomp(A);
    auto rank_A=qr_decomp.rank();
    MatrixXd B(A.rows(),yv_mapped.cols()+A.cols());
    B<<A,yv_mapped;
    qr_decomp.compute(B);
    auto rank_B=qr_decomp.rank();
    double result;
    if(rank_A==rank_B && rank_A==A.cols()){
            coeff=A.householderQr().solve(yv_mapped);
            result=(-1.0-coeff[0]*pt.x-coeff[1]*pt.y)/coeff[2];
    }
    else if(A(0,2)==A(1,2) && A(1,2)==A(2,2))
        result=A(0,2);
    else
        exitwithErrors("Error occured while predicting the disparity!");

    return result;
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


double Delaunay::calTriArea(const Delaunay::vertex *v)
{
    Point2f p1,p2,p3;
    p1=v[0].pt;
    p2=v[1].pt;
    p3=v[2].pt;
    return calTriArea(p1,p2,p3);
}


void Delaunay::convert2Contour(const Delaunay::triangle &tri, vector<Point2f> &contour)
{
    contour.clear();
    contour.push_back(tri.vtx[0].pt);
    contour.push_back(tri.vtx[1].pt);
    contour.push_back(tri.vtx[2].pt);
}


void Delaunay::generateDelaunay(const vector<Point2f> &pts,const vector<double>& attribute){
    this->insert(pts);
    getTriangulation(attribute);
    std::sort(triangulation.begin(),triangulation.end(),compTri);//descend
}


void Delaunay::generateDelaunay(const vector<KeyPoint> &kpts,const vector<double>& attribute){
    vector<Point2f> pts;
    KeyPoint2Point2f(kpts,pts);
    generateDelaunay(pts,attribute);
}


void Delaunay::generateDelaunay(const vector<Match> &matches){
    vector<double> attribute;
    for(int i=0;i<matches.size();++i){
        double attri=matches[i].p1.x-matches[i].p2.x;
        attribute.push_back(attri);
    }
    vector<Point2f> l,r;
    getPtsFromMatches(matches,l,r);
    generateDelaunay(l,attribute);
}

void Delaunay::getTriangulation(const vector<double> &attribute){
    vector<double> attri_backup=attribute;

    if(attribute.size()!=0)
        assert(attribute.size()==(this->vtx.size()-4));
    else
        attri_backup=vector<double>(this->vtx.size()-4,0);

            triangulation.clear();
            int total = (int)(this->qedges.size()*4);
            vector<bool> edgemask(total, false);
//            int idx=0;

            for(int i = 4; i < total; i += 2 ){
                if( edgemask[i] )
                    continue;
                Point2f pt[3];
                int pt_id[3];

                int edge = i;

                pt_id[0]=edgeOrg(edge, &pt[0]);
                if (pt_id[0]<4) continue;
                edgeOrg(edge, &pt[0]);
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);

                pt_id[1]=edgeOrg(edge, &pt[1]);
                if(pt_id[1]<4) continue;
                edgeOrg(edge, &pt[1]);
                edgemask[edge] = true;
                edge = getEdge(edge, NEXT_AROUND_LEFT);

                pt_id[2]=edgeOrg(edge, &pt[2]);
                if(pt_id[2]<4) continue;
                edgeOrg(edge, &pt[2]);
                edgemask[edge] = true;

                triangle tri;
//                tri.id=idx++;
                for(int k=0;k<3;++k){
                    vertex v;
                    v.id=k;
                    v.pt=pt[k];
                    v.attr=attri_backup[pt_id[k]-4];
                    tri.vtx[k]=v;
                }
                tri.area=calTriArea(tri.vtx);

                triangulation.push_back(tri);
            }
}

