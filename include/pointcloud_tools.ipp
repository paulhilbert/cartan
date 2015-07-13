template <typename PointT, template <typename> class PtrT>
template <typename MethodT>
class pointcloud_tools<PointT, PtrT>::nq_search_visitor : public boost::static_visitor<> {
    public:
        nq_search_visitor(MethodT* method);
        virtual ~nq_search_visitor();

        void operator()(float param);
        void operator()(unsigned int param);

    protected:
        MethodT*  method_;
};

template <typename PointT, template <typename> class PtrT, bool IsNormalPointType>
struct normal_dependent_algorithms_;

template <typename PointT, template <typename> class PtrT>
struct normal_dependent_algorithms_<PointT, PtrT, true> {
    typedef boost::variant<float, uint32_t> radius_or_k_t;

    static void remove_nan(PtrT<pcl::PointCloud<PointT>> cloud) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        pcl::removeNaNNormalsFromPointCloud(*cloud, *cloud, indices);
    }
    template <typename SearchT>
    static void estimate_normals(PtrT<pcl::PointCloud<PointT>> cloud, typename SearchT::Ptr search, radius_or_k_t neighborhood) {
        pcl::NormalEstimation<PointT, PointT> ne;
        ne.setInputCloud(cloud);
        ne.useSensorOriginAsViewPoint();

        ne.setSearchMethod(search);
        typename pointcloud_tools<PointT, PtrT>::template nq_search_visitor<pcl::NormalEstimation<PointT, PointT>> visitor(&ne);
        boost::apply_visitor(visitor, neighborhood);

        ne.compute(*cloud);
    }
};

template <typename PointT, template <typename> class PtrT>
struct normal_dependent_algorithms_<PointT, PtrT, false> {
    typedef boost::variant<float, uint32_t> radius_or_k_t;

    static void remove_nan(PtrT<pcl::PointCloud<PointT>> cloud) {
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
    }
    template <typename SearchT>
    static void estimate_normals(PtrT<pcl::PointCloud<PointT>> cloud, typename SearchT::Ptr search, radius_or_k_t neighborhood) {
    }
};

template <typename Point>
struct normal_tag_ : public std::false_type {};

template <> struct normal_tag_<pcl::PointNormal> : public std::true_type {};
template <> struct normal_tag_<pcl::PointXYZRGBNormal> : public std::true_type {};
template <> struct normal_tag_<pcl::PointXYZINormal> : public std::true_type {};
//template <> struct normal_tag_<pcl::PointXYZLNormal> : public std::true_type {};


template <typename PointT, template <typename> class PtrT>
inline typename pointcloud_tools<PointT, PtrT>::cloud_ptr_t
pointcloud_tools<PointT, PtrT>::from_pcd_file(const fs::path& path, bool remove_nan) {
    if (!fs::exists(path)) {
        throw std::runtime_error("Pointcloud file \"" + path.string() + "\" does not exist");
    }
    cloud_ptr_t cloud(new cloud_t());
    pcl::io::loadPCDFile(path.string(), *cloud);
    if (remove_nan) pointcloud_tools<PointT, PtrT>::remove_nan(cloud);
    return cloud;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline typename pointcloud_tools<PointT, PtrT>::cloud_ptr_t
pointcloud_tools<PointT, PtrT>::from_pcd_file(const fs::path& path, const load_options_t& options, typename SearchT::Ptr search, vec3_t* centroid) {
    cloud_ptr_t cloud = from_pcd_file(path, true);
    cloud = process_cloud_<SearchT>(cloud, options, search, centroid);
    return cloud;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline typename pointcloud_tools<PointT, PtrT>::template load_result_t<SearchT>
pointcloud_tools<PointT, PtrT>::from_pcd_file(const fs::path& path, const load_options_t& options, vec3_t* centroid) {
    typename SearchT::Ptr search(new SearchT());
    return {from_pcd_file<SearchT>(path, options, search, centroid), search};
}

#ifdef USE_LIBE57

template <typename PointT, template <typename> class PtrT>
inline std::vector<typename pointcloud_tools<PointT, PtrT>::cloud_ptr_t>
pointcloud_tools<PointT, PtrT>::from_e57_file(const fs::path& path, bool do_remove_nan) {
    if (!fs::exists(path)) {
        throw std::runtime_error("Pointcloud file \"" + path.string() + "\" does not exist");
    }

    std::vector<cloud_ptr_t> clouds;
	try {
		e57::Reader eReader(path.string());

		int scanCount = eReader.GetData3DCount();

		for (int scanIndex = 0; scanIndex < scanCount; ++scanIndex) {
			e57::Data3D scanHeader;
			eReader.ReadData3D(scanIndex, scanHeader);

			// get size info
			int64_t nColumn = 0, nRow = 0, nPointsSize = 0, nGroupsSize = 0, nCountsSize = 0, nCounts = 0; bool bColumnIndex = 0;
			eReader.GetData3DSizes( scanIndex, nRow, nColumn, nPointsSize, nGroupsSize, nCounts, bColumnIndex);

			int64_t nSize = (nRow > 0) ? nRow : 1024;

			double *xData = new double[nSize], *yData = new double[nSize], *zData = new double[nSize], *intensity = new double[nSize];

			auto dataReader = eReader.SetUpData3DPointsData(scanIndex, nSize, xData, yData, zData, NULL, intensity);

			cloud_ptr_t cloud(new cloud_t());
			float scale = 1.f;
			unsigned long size = 0;
			//float imin = scanHeader.intensityLimits.intensityMinimum;
			//float imax = scanHeader.intensityLimits.intensityMaximum;
			while((size = dataReader.read()) > 0) {
				for(unsigned long i = 0; i < size; i++) {
					point_t p;
					p.x = scale*xData[i];
					p.y = scale*yData[i];
					p.z = scale*zData[i];
					//p.intensity = (intensity[i]-imin) / (imax-imin);
					cloud->push_back(p);
				}
			}
			dataReader.close();
			Eigen::Vector4f translation(scanHeader.pose.translation.x, scanHeader.pose.translation.y, scanHeader.pose.translation.z, 0.f);
			cloud->sensor_origin_ = translation;
			Eigen::Quaternionf rotation(scanHeader.pose.rotation.w, scanHeader.pose.rotation.x, scanHeader.pose.rotation.y, scanHeader.pose.rotation.z);
			pcl::transformPointCloud(*cloud, *cloud, translation.head(3), rotation);

			delete [] xData;
			delete [] yData;
			delete [] zData;

            if (do_remove_nan) remove_nan(cloud);
            clouds.push_back(cloud);
		}
	} catch (...) {
        throw std::runtime_error("Error while loading pointcloud file \"" + path.string() + "\"");
	}

    return clouds;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline std::vector<typename pointcloud_tools<PointT, PtrT>::template load_result_t<SearchT>>
pointcloud_tools<PointT, PtrT>::from_e57_file(const fs::path& path, const load_options_t& options) {
    std::vector<cloud_ptr_t> clouds = from_e57_file(path, true);
    std::vector<load_result_t<SearchT>> result;

    vec3_t centroid = vec3_t::Zero();
    for (uint32_t i = 0; i < clouds.size(); ++i) {
        centroid *= static_cast<float>(i);
        vec3_t center;
        typename SearchT::Ptr search(new SearchT());
        clouds[i] = process_cloud_<SearchT>(clouds[i], options, search, &center);
        centroid += center;
        if (i > 0) centroid /= static_cast<float>(i+1);
        result.push_back({clouds[i], search});
    }
    if (options.demean) {
        for (auto cloud : result) {
            for (auto& p : *(cloud.first)) {
                p.getVector3fMap() -= centroid;
            }
            cloud.first->sensor_origin_.head(3) -= centroid;
        }
    }

    return result;
}

#endif // USE_LIBE57

template <typename PointT, template <typename> class PtrT>
inline std::vector<typename pointcloud_tools<PointT, PtrT>::cloud_ptr_t>
pointcloud_tools<PointT, PtrT>::from_pcd_files(const std::vector<fs::path>& paths, bool remove_nan) {
    std::vector<cloud_ptr_t> clouds(paths.size());
    std::transform(paths.begin(), paths.end(), clouds.begin(), [&] (const fs::path& p) { return from_pcd_file(p, remove_nan); });
    return clouds;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline std::vector<typename pointcloud_tools<PointT, PtrT>::template load_result_t<SearchT>>
pointcloud_tools<PointT, PtrT>::from_pcd_files(const std::vector<fs::path>& paths, const load_options_t& options) {
    std::vector<load_result_t<SearchT>> clouds(paths.size());
    vec3_t centroid = vec3_t::Zero();
    for (uint32_t i = 0; i < paths.size(); ++i) {
        centroid *= static_cast<float>(i);
        const fs::path& p = paths[i];
        vec3_t center;
        clouds[i] = from_pcd_file<SearchT>(p, options, &center);
        centroid += center;
        if (i > 0) centroid /= static_cast<float>(i+1);
    }
    if (options.demean) {
        for (auto cloud : clouds) {
            for (auto& p : *(cloud.first)) {
                p.getVector3fMap() -= centroid;
            }
            cloud.first->sensor_origin_.head(3) -= centroid;
        }
    }
    return clouds;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline typename pointcloud_tools<PointT, PtrT>::indices_t
pointcloud_tools<PointT, PtrT>::subsample(cloud_const_ptr_t cloud, typename SearchT::Ptr search, float leaf_size) {
    pcl::UniformSampling<PointT> us;
    us.setInputCloud(cloud);
    us.setSearchMethod(search);
    us.setRadiusSearch(leaf_size);
    pcl::PointCloud<int> subset_cloud;
    us.compute(subset_cloud);
    std::sort(subset_cloud.points.begin (), subset_cloud.points.end ());
    return indices_t(subset_cloud.points.begin(), subset_cloud.points.end());
}

template <typename PointT, template <typename> class PtrT>
inline typename pointcloud_tools<PointT, PtrT>::vec3_t
pointcloud_tools<PointT, PtrT>::compute_centroid(cloud_const_ptr_t cloud) {
    vec3_t centroid = vec3_t::Zero();
    uint32_t n = 0;

    for (const auto& pnt : *cloud) {
        ++n;
        vec3_t delta = pnt.getVector3fMap() - centroid;
        centroid += delta / static_cast<float>(n);
    }

    return centroid;
}

template <typename PointT, template <typename> class PtrT>
inline typename pointcloud_tools<PointT, PtrT>::vec3_t
pointcloud_tools<PointT, PtrT>::demean(cloud_ptr_t cloud) {
    vec3_t centroid = compute_centroid(cloud);
    for (auto& pnt : *cloud) {
        pnt.getVector3fMap() -= centroid;
    }
    return centroid;
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline void
pointcloud_tools<PointT, PtrT>::estimate_normals(cloud_ptr_t cloud, typename SearchT::Ptr search, radius_or_k_t neighborhood) {
    normal_dependent_algorithms_<PointT, PtrT, normal_tag_<PointT>::value>::template estimate_normals<SearchT>(cloud, search, neighborhood);
}

template <typename PointT, template <typename> class PtrT>
inline void
pointcloud_tools<PointT, PtrT>::remove_nan(cloud_ptr_t cloud) {
    normal_dependent_algorithms_<PointT, PtrT, normal_tag_<PointT>::value>::remove_nan(cloud);
}

template <typename PointT, template <typename> class PtrT>
template <typename SearchT>
inline typename pointcloud_tools<PointT, PtrT>::cloud_ptr_t
pointcloud_tools<PointT, PtrT>::process_cloud_(PtrT<pcl::PointCloud<PointT>> cloud, const load_options_t& options, typename SearchT::Ptr search, vec3_t* centroid) {
    search->setInputCloud(cloud);

    vec3_t center;
    if (options.compute_centroid_before_subsampling) {
        center = compute_centroid(cloud);
    }

    if (options.estimate_normals && options.estimate_normals_before_subsampling) {
        pointcloud_tools<PointT, PtrT>::estimate_normals<SearchT>(cloud, search, options.neighborhood);
    }

    if (options.subsample) {
        indices_t subset = subsample<SearchT>(cloud, search, options.subsample_leaf_size);
        cloud_ptr_t subset_cloud(new cloud_t(*cloud, subset));
        cloud = subset_cloud;
        search->setInputCloud(cloud);
    }

    if (options.estimate_normals && !options.estimate_normals_before_subsampling) {
        estimate_normals<SearchT>(cloud, search, options.neighborhood);
    }

    if (!options.compute_centroid_before_subsampling) {
        center = compute_centroid(cloud);
    }

    if (options.demean) {
        for (auto& p : *cloud) {
            p.getVector3fMap() -= center;
        }
        cloud->sensor_origin_.head(3) -= center;
    }

    if (centroid) {
        *centroid = center;
    }

    return cloud;
}

template <typename PointT, template <typename> class PtrT>
template <typename MethodT>
inline pointcloud_tools<PointT, PtrT>::nq_search_visitor<MethodT>::nq_search_visitor(MethodT* method) : method_(method) {
}

template <typename PointT, template <typename> class PtrT>
template <typename MethodT>
inline pointcloud_tools<PointT, PtrT>::nq_search_visitor<MethodT>::~nq_search_visitor() {
}

template <typename PointT, template <typename> class PtrT>
template <typename MethodT>
inline void
pointcloud_tools<PointT, PtrT>::nq_search_visitor<MethodT>::operator()(float param) {
    method_->setRadiusSearch(param);
}

template <typename PointT, template <typename> class PtrT>
template <typename MethodT>
inline void
pointcloud_tools<PointT, PtrT>::nq_search_visitor<MethodT>::operator()(uint32_t param) {
    method_->setKSearch(param);
}

