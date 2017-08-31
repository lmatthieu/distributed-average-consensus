cmake_minimum_required(VERSION 2.8)
project(IterativeAvgConsensus)

include(cmake-modules/ns3.cmake)


ADD_DEFINITIONS(-std=c++11)
ADD_DEFINITIONS(-DNS3_BUILD_PROFILE_DEBUG -DNS3_ASSERT_ENABLE -DNS3_LOG_ENABLE)

find_package(Boost REQUIRED COMPONENTS graph)

include_directories( ${Boost_INCLUDE_DIRS} )
INCLUDE_DIRECTORIES(${NS3_INC_DIR})
LINK_DIRECTORIES(${NS3_LIB_DIR})


add_executable(iterative_avg_consensus src/iterative_avg_consensus.cc
        src/iterative_avg_consensus.h
        src/sensor_app.h
        src/sensor_app.cc
        src/sensor_data.h src/sensor_data.cc
        src/utils.h)
add_dependencies(iterative_avg_consensus ns3)

target_link_libraries(iterative_avg_consensus
        ns${NS3_VERSION}-core-debug
        ns${NS3_VERSION}-applications-debug
        ns${NS3_VERSION}-network-debug
        ns${NS3_VERSION}-point-to-point-layout-debug
        ns${NS3_VERSION}-propagation-debug
        ns${NS3_VERSION}-internet-debug
        ns${NS3_VERSION}-mobility-debug
        ns${NS3_VERSION}-point-to-point-debug
        ns${NS3_VERSION}-energy-debug
        ns${NS3_VERSION}-wifi-debug
        ${Boost_GRAPH_LIBRARY}
        )
