include(ExternalProject)
SET(NS3_VERSION 3.26)
set(USE_NS3_DOCKER 0)

if (USE_NS3_DOCKER)
    SET(NS3_PATH /opt/ns/ns-allinone-${NS3_VERSION}/ns-${NS3_VERSION}/)
    set(NS3_INC_DIR ${NS3_PATH}/build)
    set(NS3_LIB_DIR ${NS3_PATH}/build)
else ()
    # NS3 Build
    message(STATUS "Configuring NS3")
    ExternalProject_Add(ns3
            PREFIX thirdparty/ns3
            URL https://www.nsnam.org/release/ns-allinone-${NS3_VERSION}.tar.bz2
            CONFIGURE_COMMAND cd ns-${NS3_VERSION} && ./waf configure --disable-python
            BUILD_COMMAND cd ns-${NS3_VERSION} && ./waf build
            INSTALL_COMMAND pwd
            BUILD_IN_SOURCE 1
            LOG_DOWNLOAD 1
            LOG_CONFIGURE 0
            )
    set(NS3_INC_DIR ${CMAKE_BINARY_DIR}/thirdparty/ns3/src/ns3/ns-${NS3_VERSION}/build)
    set(NS3_LIB_DIR ${CMAKE_BINARY_DIR}/thirdparty/ns3/src/ns3/ns-${NS3_VERSION}/build)
endif ()
