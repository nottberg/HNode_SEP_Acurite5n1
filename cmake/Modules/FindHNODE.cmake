include(FindPkgConfig)

pkg_check_modules(HNODE_PKG hnode-node-1.0)

if (HNODE_PKG_FOUND)
    find_path(HNODE_INCLUDE_DIR  NAMES hnode-nodeobj.h PATH_SUFFIXES hnode-1.0
       PATHS
       ${HNODE_PKG_INCLUDE_DIRS}
       /usr/include/hnode-1.0
       /usr/include
       /usr/local/include/hnode-1.0
       /usr/local/include
    )

    find_library(HNODE_LIBRARIES NAMES hnode_node
       PATHS
       ${HNODE_PKG_LIBRARY_DIRS}
       /usr/lib
       /usr/local/lib
    )

else (HNODE_PKG_FOUND)
    # Find Glib even if pkg-config is not working (eg. cross compiling to Windows)
    find_library(HNODE_LIBRARIES NAMES hnode_node)
    string (REGEX REPLACE "/[^/]*$" "" HNODE_LIBRARIES_DIR ${HNODE_LIBRARIES})

    find_path(HNODE_INCLUDE_DIR NAMES hnode-nodeobj.h PATH_SUFFIXES hnode-1.0)

endif (HNODE_PKG_FOUND)

if (HNODE_INCLUDE_DIR AND HNODE_LIBRARIES)
    set(HNODE_INCLUDE_DIRS ${HNODE_INCLUDE_DIR})
endif (HNODE_INCLUDE_DIR AND HNODE_LIBRARIES)

if(HNODE_INCLUDE_DIRS AND HNODE_LIBRARIES)
   set(HNODE_FOUND TRUE CACHE INTERNAL "hnode-1.0 found")
   message(STATUS "Found hnode-1.0: ${HNODE_INCLUDE_DIR}, ${HNODE_LIBRARIES}")
else(HNODE_INCLUDE_DIRS AND HNODE_LIBRARIES)
   set(HNODE_FOUND FALSE CACHE INTERNAL "hnode_node found")
   message(STATUS "hnode-1.0 not found.")
endif(HNODE_INCLUDE_DIRS AND HNODE_LIBRARIES)

mark_as_advanced(HNODE_INCLUDE_DIR HNODE_INCLUDE_DIRS HNODE_LIBRARIES)


