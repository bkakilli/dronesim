
include_directories(${PROJECT_SOURCE_DIR}/include)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


find_package(gazebo REQUIRED)
find_package(Threads REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_library(DepthCameraPlugin SHARED ${PROJECT_SOURCE_DIR}/src/reader/DronesimShm.cc DepthCameraPlugin.cc)
target_link_libraries(DepthCameraPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} rt)

add_library(camera_move SHARED camera_move.cc)
target_link_libraries(camera_move ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})