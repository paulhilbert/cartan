cmake_minimum_required(VERSION 2.8.8)
project(cartan)

set (cartan_VERSION_MAJOR 0)
set (cartan_VERSION_MINOR 1)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/include")
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

if(CMAKE_COMPILER_IS_GNUCXX)
    if (NOT WIN32)
        add_definitions(-fPIC)
    endif()
	add_definitions(-O3)
	add_definitions(-g)
	add_definitions(-std=c++1y)
    add_definitions(-Wall)
    add_definitions(-Wno-unused-function)
    add_definitions(-Wno-unused-local-typedefs)
endif()

find_package(Eigen)
find_package(Boost COMPONENTS date_time iostreams serialization system filesystem regex thread)

find_package(OpenMesh)
if (OPENMESH_FOUND)
    add_definitions(-DUSE_OPENMESH)
else()
    message(STATUS "No OpenMesh package found. OpenMesh support will be disabled.")
endif()

file (GLOB_RECURSE obj RELATIVE "${PROJECT_SOURCE_DIR}" "src/*.cpp")
if (EIGEN_FOUND AND Boost_FOUND)
	include_directories(${EIGEN_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
    if(OPENMESH_FOUND)
        include_directories(${OPENMESH_INCLUDE_DIRS})
    else()
        set(OPENMESH_LIBRARIES "")
    endif()

    add_library(cartan SHARED ${obj})
    target_link_libraries(cartan ${EIGEN_LIBRARIES} ${Boost_LIBRARIES} ${OPENMESH_LIBRARIES} "dl")

	# install binary
    if (WIN32)
        install (FILES "${PROJECT_BINARY_DIR}/libcartan.dll.a" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libcartan.dll" DESTINATION lib)
        install (FILES "${PROJECT_BINARY_DIR}/libcartan.dll" DESTINATION bin)
    else()
        install (TARGETS cartan DESTINATION lib)
    endif()
	# install header
    install (DIRECTORY include/ DESTINATION include/cartan)

endif()
