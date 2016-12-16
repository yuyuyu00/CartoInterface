cmake_minimum_required(VERSION 2.8.7)
project(Interface)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
aux_source_directory(. SRC)
add_executable(${PROJECT_NAME}  ${SRC})

if(MSVC)
    set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_FLAGS /DEBUG)
	include_directories(../../../cartographer/cartographer/include)
	include_directories(../Tools)
	include_directories(${LUA_INCLUDE_DIR})
	include_directories(${CERES_INCLUDE_DIRS})
	include_directories(${EIGEN3_INCLUDE_DIR})
	include_directories(${GLOG_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	
	target_link_libraries(${PROJECT_NAME} cartographer ${GFLAGS_LIBRARIES} Tools ${PCL_LIBRARIES} )
	target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${WEBP_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${LUA_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${GLOG_LIBRARIES})
	target_link_libraries(${PROJECT_NAME} ${CAIRO_LIBRARIES})


else(MSVC)
    find_package(cartographer REQUIRED HINTS "/home/yuyuyu00/desk/CartographerNew/install/share/cartographer")
	find_package(MyTools REQUIRED HINTS "/home/yuyuyu00/desk/CartographerNew/install/share/MyTools")
	FIND_LIBRARY(GFLAGS_LIBRARIES gflags /usr/lib/i386-linux-gnu)
	find_package(OpenCV  REQUIRED )
	find_package(PCL  REQUIRED )
	
	include_directories(${CARTOGRAPHER_INCLUDE_DIRS})
	include_directories(${MyTools_INCLUDE_DIRS})
	include_directories(${OPENCV_INCLUDE_DIRS})
	include_directories(${PCL_INCLUDE_DIRS})
	
	target_link_libraries(${PROJECT_NAME} ${CARTOGRAPHER_LIBRARIES} ${GFLAGS_LIBRARIES} ${MyTools_LIBS} ${PCL_LIBRARIES}  ${OpenCV_LIBS} )

endif()


	



