#Sammy Omari, Skybotix AG, 28/3/2014
cmake_minimum_required(VERSION 2.8.0)
 
############
# SETTINGS
############
set(PKGNAME vi_sensor_interface)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(Boost_USE_MULTITHREADED ON)

find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED )
find_package(VISensor REQUIRED )

set(CMAKE_BUILD_TYPE Release)
ADD_DEFINITIONS (-march=native -O2 -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x ) #Release
#ADD_DEFINITIONS (-march=native -O0 -g3 -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x ) #Debug

FILE(
  GLOB SRCS 
  src/*.cpp
)

include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories( ${Boost_INCLUDE_DIRS} )  
include_directories( ${VISensorDriver_INCLUDE_DIR} )  
include_directories(include)

add_executable(${PKGNAME} ${SRCS})
target_link_libraries(${PKGNAME} ${OpenCV_LIBS} ${VISensorDriver_LIBRARY} visensor  ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
