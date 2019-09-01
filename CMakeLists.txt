project(visual_odometry)
cmake_minimum_required(VERSION 3.10)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


#set (CMAKE_CXX_STANDARD 11)
#if(NOT CMAKE_BUILD_TYPE)
#	set(CMAKE_BUILD_TYPE Release)
#endif()


set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS "-g")
set(CMAKE_CXX_FLAGS_RELEASE '-O3')


set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/src)


add_subdirectory(src)