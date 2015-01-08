#ifndef _CARTAN_MESH_ALGORITHMS_HPP_
#define _CARTAN_MESH_ALGORITHMS_HPP_

#include <chrono>
#include <random>

#include "mesh_traits.hpp"

namespace cartan {

template <class Mesh>
class mesh_algorithms {
    public:
        typedef typename mesh_traits<Mesh>::face_handle_t  face_handle_t;
        typedef typename mesh_traits<Mesh>::position_t     position_t;
        typedef typename mesh_traits<Mesh>::eigen_vec3_t   eigen_vec3_t;

    public:
        static eigen_vec3_t centroid(const Mesh& mesh);
        static void center(Mesh& mesh);

        static float face_area(const Mesh& mesh, face_handle_t handle);
        static std::vector<float> face_areas(const Mesh& mesh);
        static float surface_area(const Mesh& mesh);

        static std::vector<std::pair<eigen_vec3_t, eigen_vec3_t>> sample_surface_points(const Mesh& mesh, uint32_t samples_per_square_unit, float normal_error = 0.f, bool deterministic = false);

    protected:
        template <class RNG>
        static eigen_vec3_t point_sample_(const Mesh& mesh, face_handle_t face_handle, RNG&& rnd01, float normal_error);
};

#include "mesh_algorithms.ipp"

} // cartan

#endif /* _CARTAN_MESH_ALGORITHMS_HPP_ */
