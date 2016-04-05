#ifndef MLS_OVERLAP
#define MLS_OVERLAP

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
        virtual void performProcessing (PointCloudOut &output,int times_of_sigma=3);
    };


    #ifdef _OPENMP
    template <typename PointInT, typename PointOutT> void
    MLS_OMP<PointInT, PointOutT>::performProcessing (PointCloudOut &output,int times_of_sigma)
    {
      // Compute the number of coefficients
      nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

      // (Maximum) number of threads
      unsigned int threads = threads_ == 0 ? 1 : threads_;

      // Create temporaries for each thread in order to avoid synchronization
      typename PointCloudOut::CloudVectorType projected_points (threads);
      typename NormalCloud::CloudVectorType projected_points_normals (threads);
      std::vector<PointIndices> corresponding_input_indices (threads);

      // For all points
    #pragma omp parallel for schedule (dynamic,1000) num_threads (threads)
      for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
      {
        // Allocate enough space to hold the results of nearest neighbor searches
        // \note resize is irrelevant for a radiusSearch ().
        std::vector<int> nn_indices;
        std::vector<float> nn_sqr_dists;

        // Get the initial estimates of point positions and their neighborhoods
        if (this->searchForNeighbors ((*indices_)[cp], nn_indices, nn_sqr_dists))
        {
          // Check the number of nearest neighbors for normal estimation (and later
          // for polynomial fit as well)
          if (nn_indices.size () >= 3)
          {
            //Check if the error is over the N times sigma
              //add something##############


            // This thread's ID (range 0 to threads-1)
            int tn = omp_get_thread_num ();

            // Size of projected points before computeMLSPointNormal () adds points
            size_t pp_size = projected_points[tn].size ();

            // Get a plane approximating the local surface's tangent and project point onto it
            int index = (*indices_)[cp];
            size_t mls_result_index = 0;

            if (upsample_method_ == VOXEL_GRID_DILATION || upsample_method_ == DISTINCT_CLOUD)
              mls_result_index = index; // otherwise we give it a dummy location.

            this->computeMLSPointNormal (index, nn_indices, nn_sqr_dists, projected_points[tn], projected_points_normals[tn], corresponding_input_indices[tn], this->mls_results_[mls_result_index]);

            // Copy all information from the input cloud to the output points (not doing any interpolation)
            for (size_t pp = pp_size; pp < projected_points[tn].size (); ++pp)
              this->copyMissingFields (input_->points[(*indices_)[cp]], projected_points[tn][pp]);
            }
          }
      }


      // Combine all threads' results into the output vectors
      for (unsigned int tn = 0; tn < threads; ++tn)
      {
        output.insert (output.end (), projected_points[tn].begin (), projected_points[tn].end ());
        corresponding_input_indices_->indices.insert (corresponding_input_indices_->indices.end (),
            corresponding_input_indices[tn].indices.begin (), corresponding_input_indices[tn].indices.end ());
        if (compute_normals_)
          normals_->insert (normals_->end (), projected_points_normals[tn].begin (), projected_points_normals[tn].end ());
      }

      // Perform the distinct-cloud or voxel-grid upsampling
      this->performUpsampling (output);
    }
    #endif

}

#endif // MLS_OVERLAP

