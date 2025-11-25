# Find the header files

FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  /usr/include
  /usr/local/include
  $ENV{G2O_ROOT}/include
)

# Find the libraries
SET(G2O_LIBS 
  g2o_core 
  g2o_stuff 
  g2o_types_sba 
  g2o_types_slam3d 
  g2o_solver_csparse 
  g2o_solver_dense 
  g2o_csparse_extension
)

FOREACH(LIB ${G2O_LIBS})
  FIND_LIBRARY(G2O_${LIB}_LIBRARY ${LIB}
    /usr/lib
    /usr/local/lib
    $ENV{G2O_ROOT}/lib
  )
  IF(G2O_${LIB}_LIBRARY)
    LIST(APPEND G2O_LIBRARIES ${G2O_${LIB}_LIBRARY})
  ENDIF()
ENDFOREACH()

IF(G2O_INCLUDE_DIR AND G2O_LIBRARIES)
  SET(G2O_FOUND TRUE)
  MESSAGE(STATUS "Found G2O: ${G2O_INCLUDE_DIR}")
ELSE()
  SET(G2O_FOUND FALSE)
  IF(G2O_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find G2O")
  ENDIF()
ENDIF()
