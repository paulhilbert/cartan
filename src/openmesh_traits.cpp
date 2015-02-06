#include <openmesh_traits.hpp>

#ifdef USE_OPENMESH


namespace cartan {

template <class ColorType>
bool mesh_traits<openmesh_t<ColorType>>::read(mesh_t& mesh, const std::string& path) {
    omerr().disable();
	OpenMesh::IO::Options opt;
	opt += OpenMesh::IO::Options::FaceNormal;
	opt += OpenMesh::IO::Options::VertexNormal;
	opt += OpenMesh::IO::Options::VertexColor;
    opt += OpenMesh::IO::Options::VertexTexCoord;

    mesh.request_vertex_colors();
	mesh.request_vertex_normals();
    mesh.request_vertex_texcoords2D();
	mesh.request_face_normals();
	bool success = OpenMesh::IO::read_mesh(mesh, path, opt);

	if (success) {
		mesh.triangulate();
		// If no face normals were loaded, estimate them.
		if (!opt.face_has_normal()) {
			mesh.update_face_normals();
		}
        mesh.update_normals();
		if (!opt.vertex_has_color()) {
			for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
				mesh.set_color(*it, ColorType(1,1,1,1));
			}
		}
	}
	return success;
}

template <class ColorType>
bool mesh_traits<openmesh_t<ColorType>>::write(const mesh_t& mesh, const std::string& path) {
	return OpenMesh::IO::write_mesh(mesh, path);
}

template <class ColorType>
uint32_t mesh_traits<openmesh_t<ColorType>>::num_vertices(const mesh_t& mesh) {
	return mesh.n_vertices();
}

template <class ColorType>
uint32_t mesh_traits<openmesh_t<ColorType>>::num_faces(const mesh_t& mesh) {
	return mesh.n_faces();
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::vertex_handle_t> mesh_traits<openmesh_t<ColorType>>::vertex_handles(const mesh_t& mesh) {
	std::vector<vertex_handle_t> vertices(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
        vertices[i++] = *it;
    }
	return vertices;
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::face_handle_t> mesh_traits<openmesh_t<ColorType>>::face_handles(const mesh_t& mesh) {
	std::vector<face_handle_t> faces;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
        faces.push_back(*it);
    }
	return faces;
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::vertex_index_t> mesh_traits<openmesh_t<ColorType>>::vertex_indices(const mesh_t& mesh) {
	std::vector<vertex_index_t> vertices(mesh.n_vertices());
	unsigned int i=0;
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
        vertices[i++] = it->idx();
    }
	return vertices;
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::face_index_t> mesh_traits<openmesh_t<ColorType>>::face_indices(const mesh_t& mesh) {
	std::vector<face_index_t> faces;
	for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
        faces.push_back(it->idx());
    }
	return faces;
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::vertex_index_t mesh_traits<openmesh_t<ColorType>>::vertex_index(const mesh_t& mesh, vertex_handle_t handle) {
    return handle.idx();
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::face_index_t mesh_traits<openmesh_t<ColorType>>::face_index(const mesh_t& mesh, face_handle_t handle) {
    return handle.idx();
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::position_t mesh_traits<openmesh_t<ColorType>>::vertex_position(const mesh_t& mesh, vertex_handle_t handle) {
	return mesh.point(handle);
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::normal_t mesh_traits<openmesh_t<ColorType>>::vertex_normal(const mesh_t& mesh, vertex_handle_t handle) {
    normal_t normal = mesh.normal(handle);
	return normal.normalized();
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::color_t mesh_traits<openmesh_t<ColorType>>::vertex_color(const mesh_t& mesh, vertex_handle_t handle) {
	return mesh.color(handle);
}

template <class ColorType>
void mesh_traits<openmesh_t<ColorType>>::set_vertex_color(mesh_t& mesh, vertex_handle_t handle, const color_t& color) {
    mesh.set_color(handle, color);
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::eigen_vec3_t mesh_traits<openmesh_t<ColorType>>::eigen_vertex_position(const mesh_t& mesh, vertex_handle_t handle) {
	return eigen_vec3_t(mesh.point(handle).data());
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::eigen_vec3_t mesh_traits<openmesh_t<ColorType>>::eigen_vertex_normal(const mesh_t& mesh, vertex_handle_t handle) {
	return eigen_vec3_t(mesh.normal(handle).data()).normalized();
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::eigen_color_t mesh_traits<openmesh_t<ColorType>>::eigen_vertex_color(const mesh_t& mesh, vertex_handle_t handle) {
    return eigen_color_t(mesh.color(handle).data());
}

template <class ColorType>
void mesh_traits<openmesh_t<ColorType>>::set_eigen_vertex_color(mesh_t& mesh, vertex_handle_t handle, const eigen_color_t& color) {
    color_t om_color(color.data());
    set_vertex_color(mesh, handle, om_color);
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::normal_t mesh_traits<openmesh_t<ColorType>>::face_normal(const mesh_t& mesh, face_handle_t handle) {
    return mesh.normal(handle);
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::eigen_vec3_t mesh_traits<openmesh_t<ColorType>>::eigen_face_normal(const mesh_t& mesh, face_handle_t handle) {
    return eigen_vec3_t(mesh.normal(handle).data());
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::vertex_handle_t> mesh_traits<openmesh_t<ColorType>>::face_vertices(const mesh_t& mesh, face_handle_t handle) {
    return std::vector<vertex_handle_t>(mesh.cfv_begin(handle), mesh.cfv_end(handle));
}

template <class ColorType>
std::vector<typename mesh_traits<openmesh_t<ColorType>>::face_handle_t> mesh_traits<openmesh_t<ColorType>>::adjacent_faces(const mesh_t& mesh, face_handle_t handle) {
	return std::vector<face_handle_t>(mesh.cff_begin(handle), mesh.cff_end(handle));
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::scalar_t mesh_traits<openmesh_t<ColorType>>::norm(const position_t& p) {
    return p.norm();
}

template <class ColorType>
typename mesh_traits<openmesh_t<ColorType>>::position_t mesh_traits<openmesh_t<ColorType>>::cross(const position_t& p0, const position_t& p1) {
    return p0 % p1;
}

template <class ColorType>
void mesh_traits<openmesh_t<ColorType>>::transform(mesh_t& mesh, const Eigen::Affine3f& transformation) {
    transform(mesh, transformation.matrix());
}

template <class ColorType>
void mesh_traits<openmesh_t<ColorType>>::transform(mesh_t& mesh, const Eigen::Matrix3f& transformation) {
	if (transformation.isIdentity()) return;

	auto normal_transform = transformation.inverse().transpose();
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
		eigen_vec3_t pos(mesh.point(*it).data());
		eigen_vec3_t nrm(mesh.normal(*it).data());
		pos = transformation*pos;
		nrm = normal_transform*nrm;
		nrm.normalize();
		mesh.set_point (*it, position_t(pos.data()));
		mesh.set_normal(*it, normal_t(nrm.data()));
	}
}

template <class ColorType>
void mesh_traits<openmesh_t<ColorType>>::transform(mesh_t& mesh, const Eigen::Matrix4f& transformation) {
	if (transformation.isIdentity()) return;

	auto normal_transform = transformation.block<3,3>(0,0).inverse().transpose();
	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
		eigen_vec4_t pos = eigen_vec3_t(mesh.point(*it).data()).homogeneous();
		eigen_vec3_t nrm(mesh.normal(*it).data());
		pos = transformation*pos;
        pos /= pos[3];
		nrm = normal_transform*nrm;
		nrm.normalize();
		mesh.set_point (*it, position_t(pos.head(3).data()));
		mesh.set_normal(*it, normal_t(nrm.data()));
	}
}

template <class ColorType>
std::shared_ptr<typename mesh_traits<openmesh_t<ColorType>>::mesh_t> mesh_traits<openmesh_t<ColorType>>::mesh_from_triangles(const polygons_t& triangles) {
    std::shared_ptr<mesh_t> mesh(new mesh_t());
    for (const auto& triangle : triangles) {
        std::vector<vertex_handle_t> handles(3);
        for (uint32_t i = 0; i < 3; ++i) {
            handles[i] = mesh->add_vertex(position_t(triangle[i].data()));
        }
        mesh->add_face(handles);
    }
    return mesh;
}

template <class ColorType>
std::shared_ptr<typename mesh_traits<openmesh_t<ColorType>>::mesh_t> mesh_traits<openmesh_t<ColorType>>::mesh_from_shared_triangles(const std::vector<eigen_vec3_t>& vertices, const std::vector<indices_t>& triangles) {
    std::shared_ptr<mesh_t> mesh(new mesh_t());
    std::vector<vertex_handle_t> handles(vertices.size());
    for (uint32_t i = 0; i < vertices.size(); ++i) {
        handles[i] = mesh->add_vertex(position_t(vertices[i].data()));
    }
    for (const auto& triangle : triangles) {
        std::vector<vertex_handle_t> triangle_handles;
        std::transform(triangle.begin(), triangle.end(), triangle_handles.begin(), [&] (uint32_t idx) { return handles[idx]; });
        mesh->add_face(triangle_handles);
    }

    return mesh;
}

template <class ColorType>
std::shared_ptr<typename mesh_traits<openmesh_t<ColorType>>::mesh_t> mesh_traits<openmesh_t<ColorType>>::mesh_from_face_subset(const mesh_t& input_mesh, std::vector<uint32_t> subset) {
    std::shared_ptr<mesh_t> mesh(new mesh_t());
    std::map<vertex_index_t, vertex_handle_t> vert_map;
    for (const auto& idx : subset) {
        auto face_iter = input_mesh.faces_begin();
        std::advance(face_iter, idx);
        std::vector<vertex_handle_t> tri_handles;
        std::vector<vertex_handle_t> verts = face_vertices(input_mesh, *face_iter);
        for (const auto& vert_handle : verts) {
            vertex_index_t vert_index = vert_handle.idx();
            if (vert_map.find(vert_index) == vert_map.end()) {
                position_t pos = input_mesh.point(vert_handle);
                normal_t nrm = input_mesh.normal(vert_handle);
                color_t col = input_mesh.color(vert_handle);
                vert_map[vert_index] = mesh->add_vertex(pos);
                mesh->set_normal(vert_map[vert_index], nrm);
                mesh->set_color(vert_map[vert_index], col);
            }
            tri_handles.push_back(vert_map[vert_index]);
        }
        mesh->add_face(tri_handles);
    }

    return mesh;
}


} // cartan

// explicit instatiation for color types
#define INSTANTIATE_COLOR_TYPE(type) \
    template struct cartan::openmesh_default_traits<type>; \
    template struct cartan::mesh_type_traits<cartan::openmesh_t<type>>; \
    template struct cartan::mesh_traits<cartan::openmesh_t<type>>;
#include <openmesh_colors.def>

#endif // USE_OPENMESH
