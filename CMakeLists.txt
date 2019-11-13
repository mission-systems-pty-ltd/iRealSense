#--------------------------------------------------------
# The CMakeLists.txt for: iRealSense
# Author(s): David Battle
#--------------------------------------------------------

cmake_minimum_required(VERSION 3.5)

SET(EXEC_NAME iRealSense)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY $ENV{BIN_HOME})

# This allows us to use the header-only JSON library
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )

# Switching on C++11 has unleashed deprecation warnings for MOOS
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations" )

#===============================================================================
# FINDING MOOS' HEADERS AND LIBRARIES...
#===============================================================================

# moos - http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php
find_package(MOOS 10.0)

message("+++++++++++++++++++++++++++++++++++++++++")
message("FIND_PACKAGE CONSIDERED:" ${MOOS_CONSIDERED_CONFIGS})
message("+++++++++++++++++++++++++++++++++++++++++")

if(NOT DEFINED MOOS_LIBRARIES)
  	message("Defining MOOS_LIBRARIES")
	set(MOOS_LIBRARIES MOOS)
endif()

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOS_INCLUDE_DIRS:" ${MOOS_INCLUDE_DIRS})
message("MOOS_LIBRARIES:   " ${MOOS_LIBRARIES})
message("+++++++++++++++++++++++++++++++++++++++++")

#===============================================================================
# FINDING MOOSGeodesy' HEADERS AND LIBRARIES...
#===============================================================================
# moos - http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php
find_package(MOOSGeodesy)

#link_directories(${MOOSGeodesy_LIBRARY_PATH})

message("+++++++++++++++++++++++++++++++++++++++++")
message("MOOSGeodesy_INCLUDE_DIRS:" ${MOOSGeodesy_INCLUDE_DIRS})
message("MOOSGeodesy_LIBRARY_PATH:" ${MOOSGeodesy_LIBRARY_PATH})
message("+++++++++++++++++++++++++++++++++++++++++")

# Some more packages
find_package( PCL 1.7.0 REQUIRED )
find_package(realsense2 REQUIRED )

add_executable(${EXEC_NAME}
    RealSense.cpp
    RealSense_Info.cpp
    main.cpp
)

target_include_directories(${EXEC_NAME} PUBLIC
	./
    $ENV{MOOSIVP_SOURCE_TREE_BASE}/include/ivp
    ${realsense_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${MOOS_INCLUDE_DIRS}
    ${MOOSGeodesy_INCLUDE_DIRS})

TARGET_LINK_LIBRARIES(${EXEC_NAME}
    ${PCL_LIBRARIES}
    realsense2
    ${MOOSGeodesy_LIBRARIES}
    ${MOOS_LIBRARIES}
    $ENV{MOOSIVP_SOURCE_TREE_BASE}/lib/libmbutil.a
    m)
