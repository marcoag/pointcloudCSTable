cmake_minimum_required(VERSION 2.6)
PROJECT( cogSub )

# Sources set
SET ( SOURCES
  main.cpp
  outlierExtraction.cpp
  cloudParticle.cpp
  cognitiveSubtraction.cpp
  icp.cpp
  QMat/qvec.cpp
  QMat/qmat.cpp
  QMat/qmatrot.cpp
  QMat/qrtmat.cpp

)
set(CMAKE_CXX_FLAGS "-Wall -Wfatal-errors -fopenmp " )

include_directories(.)

find_package(PCL 1.6 REQUIRED COMPONENTS common )
include_directories(${PCL_INCLUDE_DIRS})
include_directories(/usr/include/pcl-1.6 /usr/include/vtk-5.8)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib )
add_definitions(${PCL_DEFINITIONS})

ADD_DEFINITIONS( -msse2 -O3 )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ")

# OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
elseif(OPENMP_FOUND)
  message(FATAL shit)
endif(OPENMP_FOUND)


# Qt4
ADD_DEFINITIONS( -Wall -DQT_DLL -DQT_GUI_LIB -DQT_CORE_LIB )
FIND_PACKAGE( Qt4 REQUIRED )
INCLUDE( ${QT_USE_FILE} )

# PCL
SET ( PCL_USER_LIBRARIES  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} -lpcl_search -lpcl_common -lpcl_io -lpcl_filters -lpcl_kdtree -lpcl_octree -lpcl_segmentation -lgsl -lblas -lpcl_visualization -lvtkCommon -lvtkFiltering)

# Specify construction and link process
SET (EXECUTABLE_OUTPUT_PATH .)
ADD_EXECUTABLE( cogSub ${SOURCES} ${MOC_SOURCES} ${RC_SOURCES} ${UI_HEADERS} )
TARGET_LINK_LIBRARIES( cogSub ${LIBS} ${QT_LIBRARIES} ${PCL_USER_LIBRARIES})
INSTALL(FILES cogSub DESTINATION /usr/local/bin/ PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE )




