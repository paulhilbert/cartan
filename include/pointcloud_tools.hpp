#ifndef _CARTAN_POINTCLOUD_TOOLS_HPP_
#define _CARTAN_POINTCLOUD_TOOLS_HPP_

#include <memory>
#include <vector>
#include <type_traits>

#include <boost/variant.hpp>
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include <Eigen/Dense>

#include <pcl/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/keypoints/uniform_sampling.h>

#ifdef USE_LIBE57
#include <e57/E57Foundation.h>
#include <e57/E57Simple.h>
#endif // USE_LIBE57


namespace cartan {

template <typename PointT, template <typename> class PtrT = boost::shared_ptr>
class pointcloud_tools {
    public:
        typedef PointT                                 point_t;
        typedef pcl::PointCloud<PointT>                cloud_t;
        template <typename T>                          using ptr_t = PtrT<T>;
        template <typename T>                          using const_ptr_t = PtrT<const T>;
        typedef ptr_t<cloud_t>                         cloud_ptr_t;
        typedef const_ptr_t<cloud_t>                   cloud_const_ptr_t;
        typedef boost::variant<float, uint32_t>        radius_or_k_t;
        typedef Eigen::Vector3f                        vec3_t;
        template <typename SearchT>
        using load_result_t = std::pair<cloud_ptr_t, typename SearchT::Ptr>;
        typedef std::vector<int>                       indices_t;
        template <typename MethodT>
        class nq_search_visitor;

        typedef struct load_options__ {
            bool           subsample;
            float          subsample_leaf_size;
            bool           demean;
            bool           compute_centroid_before_subsampling;
            bool           estimate_normals;
            bool           estimate_normals_before_subsampling;
            radius_or_k_t  neighborhood;
        } load_options_t;

    public:
        static cloud_ptr_t from_pcd_file(const fs::path& path, bool remove_nan = true);
        template <typename SearchT>
        static cloud_ptr_t from_pcd_file(const fs::path& path, const load_options_t& options, typename SearchT::Ptr search, vec3_t* centroid = nullptr);
        template <typename SearchT>
        static load_result_t<SearchT> from_pcd_file(const fs::path& path, const load_options_t& options, vec3_t* centroid = nullptr);
#ifdef USE_LIBE57
        static std::vector<cloud_ptr_t> from_e57_file(const fs::path& path, bool remove_nan = true);
        template <typename SearchT>
        static std::vector<load_result_t<SearchT>> from_e57_file(const fs::path& path, const load_options_t& options);
#endif // USE_LIBE57

        static std::vector<cloud_ptr_t> from_pcd_files(const std::vector<fs::path>& paths, bool remove_nan = true);
        template <typename SearchT>
        static std::vector<load_result_t<SearchT>> from_pcd_files(const std::vector<fs::path>& paths, const load_options_t& options);

        template <typename SearchT>
        static indices_t subsample(cloud_const_ptr_t cloud, typename SearchT::Ptr search, float leaf_size);
        static vec3_t compute_centroid(cloud_const_ptr_t cloud);
        static vec3_t demean(cloud_ptr_t cloud);
        template <typename SearchT>
        static void   estimate_normals(cloud_ptr_t cloud, typename SearchT::Ptr search, radius_or_k_t neighborhood);
        static void   remove_nan(cloud_ptr_t cloud);

    protected:
        template <typename SearchT>
        static cloud_ptr_t process_cloud_(PtrT<pcl::PointCloud<PointT>> cloud, const load_options_t& options, typename SearchT::Ptr search, vec3_t* centroid);
};


#include "pointcloud_tools.ipp"

} // cartan

#endif /* _CARTAN_POINTCLOUD_TOOLS_HPP_ */
