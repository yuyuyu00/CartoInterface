cmake_minimum_required(VERSION 2.8.7)

project(test1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#SET(CMAKE_MODULE_PATH "/home/yuyuyu00/desk/CartographerNew/install/share/cartographer/") 
#set(Cartographer_DIR "/home/yuyuyu00/desk/CartographerNew/install/")
#SET(CMAKE_MODULE_PATH )

find_package(cartographer REQUIRED HINTS "/home/yuyuyu00/desk/CartographerNew/install/share/cartographer")

find_package(MyTools REQUIRED HINTS "/home/yuyuyu00/desk/CartographerNew/install/share/MyTools")


message("cmake---"  ${CMAKE_MODULE_PATH} )


message("include---"  ${CARTOGRAPHER_INCLUDE_DIRS} )

FIND_LIBRARY(GFLAGS_LIBRARIES gflags /usr/lib/i386-linux-gnu)

aux_source_directory(. SRC)
add_executable(test1 ${SRC})

include_directories(${CARTOGRAPHER_INCLUDE_DIRS})
include_directories(${MyTools_INCLUDE_DIRS})
message("includemytools---"  ${MyTools_INCLUDE_DIRS} )


target_link_libraries(test1 ${CARTOGRAPHER_LIBRARIES})
message("current---"  ${CARTOGRAPHER_LIBRARIES} )
target_link_libraries(test1 ${GFLAGS_LIBRARIES} ${MyTools_LIBS} )



install(TARGETS test1 RUNTIME DESTINATION bin)
