#ifndef _CARTAN_OPENMESH_COLOR_TRAITS_HPP_
#define _CARTAN_OPENMESH_COLOR_TRAITS_HPP_

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>

namespace cartan {

template <class ColorType>
struct openmesh_color_traits; // { typedef ... scalar_t; static constexpr dim = ... }

template <>
struct openmesh_color_traits<OpenMesh::Vec3uc> {
    typedef unsigned char scalar_t;
    static constexpr int dim = 3;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec4uc> {
    typedef unsigned char scalar_t;
    static constexpr int dim = 4;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec3ui> {
    typedef unsigned int scalar_t;
    static constexpr int dim = 3;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec4ui> {
    typedef unsigned int scalar_t;
    static constexpr int dim = 4;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec3i> {
    typedef int scalar_t;
    static constexpr int dim = 3;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec4i> {
    typedef int scalar_t;
    static constexpr int dim = 4;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec3f> {
    typedef float scalar_t;
    static constexpr int dim = 3;
};

template <>
struct openmesh_color_traits<OpenMesh::Vec4f> {
    typedef float scalar_t;
    static constexpr int dim = 4;
};


} // cartan

#endif /* _CARTAN_OPENMESH_COLOR_TRAITS_HPP_ */
