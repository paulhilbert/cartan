###############################################################################
# Find Cartan
#
# This sets the following variables:
# CARTAN_FOUND - True if CARTAN was found.
# CARTAN_INCLUDE_DIRS - Directories containing the CARTAN include files.
# CARTAN_LIBRARY_DIRS - Directories containing the CARTAN library.
# CARTAN_LIBRARIES - CARTAN library files.

if(WIN32)
    find_path(CARTAN_INCLUDE_DIR e57_pcl PATHS "/usr/include" "/usr/local/include" "/usr/x86_64-w64-mingw32/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)

    find_library(CARTAN_LIBRARY_PATH e57_pcl PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${CARTAN_LIBRARY_PATH})
        get_filename_component(CARTAN_LIBRARY ${CARTAN_LIBRARY_PATH} NAME)
        find_path(CARTAN_LIBRARY_DIR ${CARTAN_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" "/usr/x86_64-w64-mingw32/lib" NO_DEFAULT_PATHS)
    endif()
else(WIN32)
    find_path(CARTAN_INCLUDE_DIR e57_pcl PATHS "/usr/include" "/usr/local/include" "$ENV{PROGRAMFILES}" NO_DEFAULT_PATHS)
    find_library(CARTAN_LIBRARY_PATH e57_pcl PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)

    if(EXISTS ${CARTAN_LIBRARY_PATH})
        get_filename_component(CARTAN_LIBRARY ${CARTAN_LIBRARY_PATH} NAME)
        find_path(CARTAN_LIBRARY_DIR ${CARTAN_LIBRARY} PATHS "/usr/lib" "/usr/local/lib" NO_DEFAULT_PATHS)
    endif()
endif(WIN32)

set(CARTAN_INCLUDE_DIRS ${CARTAN_INCLUDE_DIR})
set(CARTAN_LIBRARY_DIRS ${CARTAN_LIBRARY_DIR})
set(CARTAN_LIBRARIES ${CARTAN_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CARTAN DEFAULT_MSG CARTAN_INCLUDE_DIR CARTAN_LIBRARY CARTAN_LIBRARY_DIR)

mark_as_advanced(CARTAN_INCLUDE_DIR)
mark_as_advanced(CARTAN_LIBRARY_DIR)
mark_as_advanced(CARTAN_LIBRARY)
mark_as_advanced(CARTAN_LIBRARY_PATH)
