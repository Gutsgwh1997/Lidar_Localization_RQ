cmake_minimum_required(VERSION 2.8.3)
project(lidar_localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  geometry_msgs
  tf
  eigen_conversions
  message_generation
  std_srvs
)

add_service_files(
    FILES
    saveMap.srv
    optimizeMap.srv
    )

generate_messages(
    DEPENDENCIES
    std_msgs
    )

set(ALL_TARGET_LIBRARIES "")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/eigen.cmake)
include(cmake/geographic.cmake)
include(cmake/global_defination.cmake)
include(cmake/YAML.cmake)
include(cmake/g2o.cmake)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_package()

file(GLOB_RECURSE ALL_SRCS "*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/app/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
file(GLOB_RECURSE TEST_SRCS "test/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})
list(REMOVE_ITEM ALL_SRCS ${TEST_SRCS})

add_executable(test_frame_node src/app/test_frame_node.cpp ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/app/front_end_node.cpp ${ALL_SRCS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
add_dependencies(front_end_node ${catkin_EXPORTED_TARGETS})

add_executable(data_preprocessing_node src/app/data_preprocessing_node.cpp ${ALL_SRCS})
target_link_libraries(data_preprocessing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(data_pretreat_node src/app/data_pretreat_node.cpp ${ALL_SRCS})
target_link_libraries(data_pretreat_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(back_end_node src/app/back_end_node.cpp ${ALL_SRCS})
add_dependencies(back_end_node ${catkin_EXPORTED_TARGETS} optimizeMap_gencpp)
target_link_libraries(back_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(loop_closing_node src/app/loop_closing_node.cpp ${ALL_SRCS})
add_dependencies(loop_closing_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(loop_closing_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(viewer_node src/app/viewer_node.cpp ${ALL_SRCS})
add_dependencies(viewer_node ${catkin_EXPORTED_TARGETS} saveMap_gencpp)
target_link_libraries(viewer_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})


# 我的测试文件
add_executable(read_trajectory_and_mapping test/read_trajectory_and_mapping.cpp ${ALL_SRCS})
add_dependencies(read_trajectory_and_mapping ${catkin_EXPORTED_TARGETS})
target_link_libraries(read_trajectory_and_mapping ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})
