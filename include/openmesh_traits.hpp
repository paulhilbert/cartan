#ifndef _CARTAN_OPENMESH_TRAITS_HPP_
#define _CARTAN_OPENMESH_TRAITS_HPP_

#include "mesh_traits.hpp"
#include "openmesh_color_traits.hpp"

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
//#include <OpenMesh/Core/Utils/color_cast.hh>

namespace cartan {

template <class ColorType>
struct openmesh_default_traits : public OpenMesh::DefaultTraits {
	typedef ColorType  Color;

	VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
	FaceAttributes( OpenMesh::Attributes::Normal );
};

template <class ColorType>
using openmesh_t = ::OpenMesh::TriMesh_ArrayKernelT<openmesh_default_traits<ColorType>>;


template <class ColorType>
struct mesh_type_traits<openmesh_t<ColorType>> {
	static constexpr int color_dim = openmesh_color_traits<ColorType>::dim;

	typedef openmesh_t<ColorType>                         mesh_t;
	typedef float                                         scalar_t;
	typedef typename openmesh_t<ColorType>::Point         position_t;
	typedef typename openmesh_t<ColorType>::Normal        normal_t;
	typedef ColorType                                     color_t;
	typedef typename openmesh_t<ColorType>::VertexHandle  vertex_handle_t;
	typedef typename openmesh_t<ColorType>::FaceHandle    face_handle_t;
	typedef typename openmesh_t<ColorType>::EdgeHandle    edge_handle_t;
	typedef uint32_t                                      vertex_index_t;
	typedef uint32_t                                      face_index_t;
    typedef openmesh_color_traits<color_t>                color_traits_t;
    typedef typename color_traits_t::scalar_t             color_scalar_t;
	typedef Eigen::Matrix<scalar_t, 3, 1>                 eigen_vec3_t;
	typedef Eigen::Matrix<scalar_t, 4, 1>                 eigen_vec4_t;
    typedef Eigen::Matrix<color_scalar_t, color_dim, 1>   eigen_color_t;
    typedef std::vector<uint32_t>                         indices_t;
    typedef std::vector<eigen_vec3_t>                     polygon_t;
    typedef std::vector<polygon_t>                        polygons_t;
};


template <class ColorType>
struct mesh_traits<openmesh_t<ColorType>> {
	static constexpr int color_dim = mesh_type_traits<openmesh_t<ColorType>>::color_dim;

	typedef typename mesh_type_traits<openmesh_t<ColorType>>::mesh_t           mesh_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::scalar_t         scalar_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::position_t       position_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::normal_t         normal_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::color_t          color_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::vertex_handle_t  vertex_handle_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::face_handle_t    face_handle_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::edge_handle_t    edge_handle_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::vertex_index_t   vertex_index_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::face_index_t     face_index_t;
    typedef typename mesh_type_traits<openmesh_t<ColorType>>::color_scalar_t   color_scalar_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::eigen_vec3_t     eigen_vec3_t;
	typedef typename mesh_type_traits<openmesh_t<ColorType>>::eigen_vec4_t     eigen_vec4_t;
    typedef typename mesh_type_traits<openmesh_t<ColorType>>::eigen_color_t    eigen_color_t;
    typedef typename mesh_type_traits<openmesh_t<ColorType>>::indices_t        indices_t;
    typedef typename mesh_type_traits<openmesh_t<ColorType>>::polygon_t        polygon_t;
    typedef typename mesh_type_traits<openmesh_t<ColorType>>::polygons_t       polygons_t;

	static bool read(mesh_t& mesh, const std::string& path);
	static bool write(const mesh_t& mesh, const std::string& path);

    static void from_polygons(mesh_t& mesh, const polygons_t& polygons, const polygons_t& vertex_normals = polygons_t(), bool triangulate = true, const std::vector<eigen_vec4_t>& colors = std::vector<eigen_vec4_t>());

	static uint32_t num_vertices(const mesh_t& mesh);
	static uint32_t num_faces(const mesh_t& mesh);

	static std::vector<vertex_handle_t> vertex_handles(const mesh_t& mesh);
	static std::vector<face_handle_t> face_handles(const mesh_t& mesh);
	static std::vector<edge_handle_t> edge_handles(const mesh_t& mesh);

	static std::vector<vertex_index_t> vertex_indices(const mesh_t& mesh);
	static std::vector<face_index_t> face_indices(const mesh_t& mesh);

	static vertex_index_t vertex_index(const mesh_t& mesh, vertex_handle_t handle);
	static face_index_t face_index(const mesh_t& mesh, face_handle_t handle);
	static position_t vertex_position(const mesh_t& mesh, vertex_handle_t handle);
	static normal_t vertex_normal(const mesh_t& mesh, vertex_handle_t handle);
	static color_t vertex_color(const mesh_t& mesh, vertex_handle_t handle);
	static void set_vertex_color(mesh_t& mesh, vertex_handle_t handle, const color_t& color);
	static eigen_vec3_t eigen_vertex_position(const mesh_t& mesh, vertex_handle_t handle);
	static eigen_vec3_t eigen_vertex_normal(const mesh_t& mesh, vertex_handle_t handle);
    static eigen_color_t eigen_vertex_color(const mesh_t& mesh, vertex_handle_t handle);
    static void set_eigen_vertex_color(mesh_t& mesh, vertex_handle_t handle, const eigen_color_t& color);
	static void set_eigen_vertex_position(mesh_t& mesh, vertex_handle_t handle, const eigen_vec3_t& pos);

    static std::pair<face_handle_t, face_handle_t> edge_faces(const mesh_t& mesh, edge_handle_t edge);
    static std::pair<vertex_handle_t, vertex_handle_t> edge_vertices(const mesh_t& mesh, edge_handle_t edge);

	static normal_t face_normal(const mesh_t& mesh, face_handle_t id);
	static eigen_vec3_t eigen_face_normal(const mesh_t& mesh, face_handle_t id);

	static std::vector<vertex_handle_t> face_vertices(const mesh_t& mesh, face_handle_t face);
	static std::vector<face_handle_t> adjacent_faces(const mesh_t& mesh, face_handle_t face);

	static scalar_t norm(const position_t& p);
	static position_t cross(const position_t& p0, const position_t& p1);

	static void transform(mesh_t& mesh, const Eigen::Affine3f& transformation);
	static void transform(mesh_t& mesh, const Eigen::Matrix3f& transformation);
	static void transform(mesh_t& mesh, const Eigen::Matrix4f& transformation);

    static std::shared_ptr<mesh_t> mesh_from_triangles(const polygons_t& triangles);
    static std::shared_ptr<mesh_t> mesh_from_shared_triangles(const std::vector<eigen_vec3_t>& vertices, const std::vector<indices_t>& triangles);
    static std::shared_ptr<mesh_t> mesh_from_face_subset(const mesh_t& input_mesh, std::vector<uint32_t> subset);
};



} // cartan

#endif /* _CARTAN_OPENMESH_TRAITS_HPP_ */
