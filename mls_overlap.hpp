#ifndef MLS_OVERLAP
#define MLS_OVERLAP

#include <iostream>
#include <math.h>

#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/search/kdtree.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/surface/mls.h"
#include "pcl/surface/impl/mls.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace pcl{
    template <typename PointInT, typename PointOutT>
    class MLS_OMP:public MovingLeastSquaresOMP<PointInT, PointOutT>{
    public:
        using MovingLeastSquaresOMP<PointInT, PointOutT>::input_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::indices_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::normals_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::corresponding_input_indices_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::nr_coeff_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::order_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::compute_normals_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::upsample_method_;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::VOXEL_GRID_DILATION;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::DISTINCT_CLOUD;
        using MovingLeastSquaresOMP<PointInT, PointOutT>::threads_;

        typedef pcl::PointCloud<pcl::Normal> NormalCloud;
        typedef pcl::PointCloud<pcl::Normal>::Ptr NormalCloudPtr;

        typedef pcl::PointCloud<PointOutT> PointCloudOut;
        typedef typename PointCloudOut::Ptr PointCloudOutPtr;
        typedef typename PointCloudOut::ConstPtr PointCloudOutConstPtr;
        typedef boost::shared_ptr<MovingLeastSquares<PointInT, PointOutT> > Ptr;
        typedef boost::shared_ptr<const MovingLeastSquares<PointInT, PointOutT> > ConstPtr;
//        virtual void performProcessing (PointCloudOut &output,int times_of_sigma=3);
        inline int new_searchForNeighbors (int index, std::vector<int> &indices, std::vector<float> &sqr_distances) const
        {
            return MovingLeastSquaresOMP<PointInT, PointOutT>::searchForNeighbors(index,indices,sqr_distances);
        }
    };
}

#endif // MLS_OVERLAP

