cmake_minimum_required(VERSION 3.0.0)
project(lidarpoint VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
enable_testing()
include_directories(${CMAKE_SOURCE_DIR}/include)


#----LIBRARIES--------------------------------------------------------------
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED QUIET)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#----LIBRARIES--------------------------------------------------------------


#----LIB--------------------------------------------------------------

ADD_LIBRARY (preprocess src/preprocess.cpp)
target_link_libraries(preprocess  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenCV_LIBRARIES})
#----LIB--------------------------------------------------------------



#----main--------------------------------------------------------------


add_executable(lidarpoint main.cpp include/colorprint.cpp )
target_link_libraries(lidarpoint  preprocess ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(bin2pcd src/bin2pcd.cpp )
target_link_libraries(bin2pcd   ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})




#----main--------------------------------------------------------------

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
