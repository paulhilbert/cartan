template <class Mesh>
inline typename mesh_algorithms<Mesh>::eigen_vec3_t mesh_algorithms<Mesh>::centroid(const Mesh& mesh) {
    uint32_t n = 0;
    eigen_vec3_t centroid = eigen_vec3_t::Zero(), delta;

    for (const auto& v : mesh_traits<Mesh>::vertex_handles(mesh)) {
        eigen_vec3_t p = mesh_traits<Mesh>::eigen_vertex_position(mesh, v);
        ++n;
        delta = p - centroid;
        centroid += delta / static_cast<float>(n);
    }

    return centroid;
}

template <class Mesh>
inline void mesh_algorithms<Mesh>::center(Mesh& mesh) {
    eigen_vec3_t c = mesh_algorithms<Mesh>::centroid(mesh);

    for (const auto& v : mesh_traits<Mesh>::vertex_handles(mesh)) {
        eigen_vec3_t p = mesh_traits<Mesh>::eigen_vertex_position(mesh, v);
        mesh_traits<Mesh>::set_eigen_vertex_position(mesh, v, p - c);
    }
}

template <class Mesh>
inline float mesh_algorithms<Mesh>::face_area(const Mesh& mesh, face_handle_t handle) {
    auto vertices = mesh_traits<Mesh>::face_vertices(mesh, handle);
    auto d0 = mesh_traits<Mesh>::vertex_position(mesh, vertices[1]) - mesh_traits<Mesh>::vertex_position(mesh, vertices[0]);
    auto d1 = mesh_traits<Mesh>::vertex_position(mesh, vertices[2]) - mesh_traits<Mesh>::vertex_position(mesh, vertices[0]);
    return static_cast<float>(0.5 * mesh_traits<Mesh>::norm(mesh_traits<Mesh>::cross(d0, d1)));
}

template <class Mesh>
inline std::vector<float> mesh_algorithms<Mesh>::face_areas(const Mesh& mesh) {
    auto faces = mesh_traits<Mesh>::face_handles(mesh);
    std::vector<float> areas(faces.size());
    std::transform(faces.begin(), faces.end(), areas.begin(), [&] (face_handle_t handle) { return face_area(mesh, handle); });
    return areas;
}

template <class Mesh>
inline float mesh_algorithms<Mesh>::surface_area(const Mesh& mesh) {
    auto areas = face_areas(mesh);
    return std::accumulate(areas.begin(), areas.end(), 0.f);
}

template <class Mesh>
inline std::vector<std::pair<typename mesh_algorithms<Mesh>::eigen_vec3_t, typename mesh_algorithms<Mesh>::eigen_vec3_t>> mesh_algorithms<Mesh>::sample_surface_points(const Mesh& mesh, float samples_per_square_unit, float normal_error, bool deterministic) {
    std::vector<float> hist = face_areas(mesh);
	std::partial_sum(hist.begin(), hist.end(), hist.begin());
	float surface_area = hist.back();
	uint32_t num_samples = static_cast<uint32_t>(surface_area * static_cast<float>(samples_per_square_unit));

    uint32_t seed = deterministic ? 0 : std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<float> gen01;
    auto rng01 = std::bind(std::uniform_real_distribution<float>(), generator);

	// sample
	std::vector<std::pair<eigen_vec3_t, eigen_vec3_t>> samples(num_samples);
    auto face_handles = mesh_traits<Mesh>::face_handles(mesh);
	for (auto it = samples.begin(); it != samples.end(); ++it) {
		auto area_iter = std::lower_bound(hist.begin(), hist.end(), surface_area * rng01());
		auto face_iter = face_handles.begin();
		std::advance(face_iter, std::distance(hist.begin(), area_iter));
		*it = std::make_pair(point_sample_(mesh, *face_iter, rng01, normal_error), mesh_traits<Mesh>::eigen_face_normal(mesh, *face_iter));
	}

	return samples;
}

template <class Mesh>
template <class RNG>
inline typename mesh_algorithms<Mesh>::eigen_vec3_t mesh_algorithms<Mesh>::point_sample_(const Mesh& mesh, face_handle_t face_handle, RNG&& rnd01, float normal_error) {
	float s = std::sqrt(rnd01());
	float t = rnd01();
	float st = s*t;
	auto vertices = mesh_traits<Mesh>::face_vertices(mesh, face_handle);
	std::vector<eigen_vec3_t> positions(vertices.size());
	std::transform(vertices.begin(), vertices.end(), positions.begin(), [&] (typename mesh_traits<Mesh>::vertex_handle_t vert) { return mesh_traits<Mesh>::eigen_vertex_position(mesh, vert); });
    eigen_vec3_t pos = (1.f - s) * positions[0] + (s - st) * positions[1] + st * positions[2];
    eigen_vec3_t normal = mesh_traits<Mesh>::eigen_face_normal(mesh, face_handle);
    pos += rnd01() * normal_error * normal;
    return pos;
}
