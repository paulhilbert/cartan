#ifndef _CARTAN_MESH_TRAITS_HPP_
#define _CARTAN_MESH_TRAITS_HPP_

#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <Eigen/Dense>

#include "eigen_color_cast.hpp"

namespace cartan {

template <class Mesh>
struct mesh_type_traits;

template <class Mesh>
struct mesh_traits {
	static constexpr int color_dim = mesh_type_traits<Mesh>::dim;

	typedef typename mesh_type_traits<Mesh>::mesh_t            mesh_t;
	typedef typename mesh_type_traits<Mesh>::scalar_t          scalar_t;
	typedef typename mesh_type_traits<Mesh>::position_t        position_t;
	typedef typename mesh_type_traits<Mesh>::normal_t          normal_t;
	typedef typename mesh_type_traits<Mesh>::color_t           color_t;
	typedef typename mesh_type_traits<Mesh>::vertex_handle_t   vertex_handle_t;
	typedef typename mesh_type_traits<Mesh>::face_handle_t     face_handle_t;
	typedef typename mesh_type_traits<Mesh>::vertex_index_t    vertex_index_t;
	typedef typename mesh_type_traits<Mesh>::face_index_t      face_index_t;
    typedef typename mesh_type_traits<Mesh>::color_scalar_t    color_scalar_t;
	typedef typename mesh_type_traits<Mesh>::eigen_vec3_t      eigen_vec3_t;
	typedef typename mesh_type_traits<Mesh>::eigen_vec4_t      eigen_vec4_t;
    typedef typename mesh_type_traits<Mesh>::eigen_color_t     eigen_color_t;

	static bool read(mesh_t& mesh, const std::string& path);
	static bool write(const mesh_t& mesh, const std::string& path);

	static uint32_t num_vertices(const mesh_t& mesh);
	static uint32_t num_faces(const mesh_t& mesh);

	static std::vector<vertex_handle_t> vertex_handles(const mesh_t& mesh);
	static std::vector<face_handle_t> face_handles(const mesh_t& mesh);

	static vertex_index_t vertex_index(const mesh_t& mesh, vertex_handle_t handle);
	static face_index_t face_index(const mesh_t& mesh, face_handle_t handle);
	static position_t vertex_position(const mesh_t& mesh, vertex_handle_t handle);
	static normal_t vertex_normal(const mesh_t& mesh, vertex_handle_t handle);
	static color_t vertex_color(const mesh_t& mesh, vertex_handle_t handle);
	static void set_vertex_color(mesh_t& mesh, vertex_handle_t handle, const color_t& color);
	static eigen_vec3_t eigen_vertex_position(const mesh_t& mesh, vertex_handle_t handle);
	static eigen_vec4_t eigen_vertex_normal(const mesh_t& mesh, vertex_handle_t handle);
    static eigen_color_t eigen_vertex_color(const mesh_t& mesh, vertex_handle_t handle);
    static void set_eigen_vertex_color(mesh_t& mesh, vertex_handle_t handle, const eigen_color_t& color);

	static normal_t face_normal(const mesh_t& mesh, face_handle_t handle);
	static eigen_vec3_t eigen_face_normal(const mesh_t& mesh, face_handle_t handle);

	static std::vector<vertex_handle_t> face_vertices(const mesh_t& mesh, face_handle_t handle);
	static std::vector<face_handle_t> adjacent_faces(const mesh_t& mesh, face_handle_t handle);

	static scalar_t norm(const position_t& p);
	static position_t cross(const position_t& p0, const position_t& p1);

	static void transform(mesh_t& mesh, const Eigen::Affine3f& transformation);
	static void transform(mesh_t& mesh, const Eigen::Matrix3f& transformation);
	static void transform(mesh_t& mesh, const Eigen::Matrix4f& transformation);
};

} // cartan

#endif /* _CARTAN_MESH_TRAITS_HPP_ */
