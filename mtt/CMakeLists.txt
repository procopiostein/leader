cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Graphviz
find_path(GRAPHVIZ_INCLUDE_DIR graphviz/gvc.h)
find_library(GRAPHVIZ_GVC_LIBRARY NAMES gvc )
find_library(GRAPHVIZ_GRAPH_LIBRARY NAMES graph )
find_package(GTK2 2.6 REQUIRED gtk)

set(DO_MHT false)

if(${GRAPHVIZ_INCLUDE_DIR} STREQUAL GRAPHVIZ_INCLUDE_DIR-NOTFOUND )
	message(WARNING "Graphviz is not installed please do: sudo apt-get install libgraphviz-dev")
	set(DO_MHT false)
endif()

if(${GRAPHVIZ_GVC_LIBRARY} STREQUAL GRAPHVIZ_GVC_LIBRARY-NOTFOUND )
	message(WARNING "GVC library is not installed please do: sudo apt-get install libgraphviz-dev")
	set(DO_MHT false)
endif()

if(${GRAPHVIZ_GRAPH_LIBRARY} STREQUAL GRAPHVIZ_GRAPH_LIBRARY-NOTFOUND )
	message(WARNING "Graph library is not installed please do: sudo apt-get install libgraphviz-dev")
	set(DO_MHT false)
endif()

if(NOT GTK2_FOUND)
	message(WARNING "Gtk library is not installed please do: sudo apt-get install libgtk2.0-dev")
	set(DO_MHT false)
endif()

rosbuild_genmsg()
rosbuild_add_library(colormap src/colormap.cpp)
rosbuild_add_executable(mtt_new_msg src/mtt_new_msg.cpp src/mtt_clustering.cpp src/mtt_auxiliar.cpp src/mtt_association.cpp src/mtt_kalman.cpp)
rosbuild_add_executable(simple_planar_pc_generator_atlasmv src/simple_planar_pc_generator_atlasmv.cpp)
target_link_libraries(mtt_new_msg colormap)
