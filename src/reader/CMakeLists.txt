set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

project(reader)

find_package(PCL 1.2 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )

add_executable (reader DronesimShm.cc reader.cpp)
target_link_libraries (reader ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} rt)