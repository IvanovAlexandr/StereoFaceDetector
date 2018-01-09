cmake_minimum_required(VERSION 2.8)
project(image_view)

find_package(catkin REQUIRED COMPONENTS camera_calibration_parsers cv_bridge dynamic_reconfigure image_transport message_filters message_generation nodelet rosconsole roscpp std_srvs stereo_msgs)
generate_dynamic_reconfigure_options(cfg/ImageView.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure)
find_package(Boost REQUIRED COMPONENTS signals thread)
#find_package(OpenCV REQUIRED)

find_package(OpenCV REQUIRED
  NO_MODULE # should be optional, tells CMake to use config mode
  PATHS /usr/local # look here
  NO_DEFAULT_PATH
) # and don't look anywhere else

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

# Deal with the GUI's
if(ANDROID)
  return()
endif()

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

# Nodelet library
add_library(image_view src/nodelets/image_nodelet.cpp src/nodelets/disparity_nodelet.cpp src/nodelets/window_thread.cpp)
target_link_libraries(image_view ${catkin_LIBRARIES}
                                 ${GTK_LIBRARIES}
                                 ${GTK2_LIBRARIES}
                                 ${OpenCV_LIBRARIES}
                                 ${Boost_LIBRARIES}
)
install(TARGETS image_view
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Image viewers

add_executable(stereo_view src/nodes/stereo_view.cpp)
target_link_libraries(stereo_view ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
                                  ${GTK_LIBRARIES}
                                  ${GTK2_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)

add_executable(test_view src/nodes/test_view.cpp)
target_link_libraries(test_view ${Boost_LIBRARIES}
                                  ${catkin_LIBRARIES}
                                  ${GTK_LIBRARIES}
                                  ${GTK2_LIBRARIES}
                                  ${OpenCV_LIBRARIES}
)

install(TARGETS stereo_view
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)