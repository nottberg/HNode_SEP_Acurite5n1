include(FindPkgConfig)

pkg_check_modules(HNREST_PKG hnode-rest-1.0)

if (HNREST_PKG_FOUND)
    find_path(HNREST_INCLUDE_DIR  NAMES hnode-rest.hpp PATH_SUFFIXES hnode-1.0
       PATHS
       ${HNREST_PKG_INCLUDE_DIRS}
       /usr/include/hnode-1.0
       /usr/include
       /usr/local/include/hnode-1.0
       /usr/local/include
    )

    find_library(HNREST_LIBRARIES NAMES hnode_rest
       PATHS
       ${HNREST_PKG_LIBRARY_DIRS}
       /usr/lib
       /usr/local/lib
    )
else (HNREST_PKG_FOUND)
    # Find Glib even if pkg-config is not working (eg. cross compiling to Windows)
    find_library(HNREST_LIBRARIES NAMES hnode_rest)

    string (REGEX REPLACE "/[^/]*$" "" HNREST_LIBRARIES_DIR ${HNREST_LIBRARIES})

    find_path(HNREST_INCLUDE_DIR NAMES hnode-rest.hpp PATH_SUFFIXES hnode-1.0)

endif (HNREST_PKG_FOUND)

if (HNREST_INCLUDE_DIR AND HNREST_LIBRARIES)
    set(HNREST_INCLUDE_DIRS ${HNREST_INCLUDE_DIR})
endif (HNREST_INCLUDE_DIR AND HNREST_LIBRARIES)

if(HNREST_INCLUDE_DIRS AND HNREST_LIBRARIES)
   set(HNREST_FOUND TRUE CACHE INTERNAL "hnode-rest-1.0 found")
   message(STATUS "Found hnode-rest-1.0: ${HNREST_INCLUDE_DIR}, ${HNREST_LIBRARIES}")
else(HNREST_INCLUDE_DIRS AND HNREST_LIBRARIES)
   set(HNREST_FOUND FALSE CACHE INTERNAL "hnode_rest found")
   message(STATUS "hnode-rest-1.0 not found.")
endif(HNREST_INCLUDE_DIRS AND HNREST_LIBRARIES)

mark_as_advanced(HNREST_INCLUDE_DIR HNREST_INCLUDE_DIRS HNREST_LIBRARIES)


