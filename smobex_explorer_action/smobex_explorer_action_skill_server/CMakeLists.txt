cmake_minimum_required(VERSION 2.8.3)
project(smobex_explorer_action_skill_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  smobex_explorer_action_skill_msgs
  colormap
  smobex_explorer
)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

file(GLOB PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "include/${PROJECT_NAME}/*.h")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS smobex_explorer
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${colormap_INCLUDE_DIRS}
  ${smobex_explorer_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/smobex_explorer_action_skill_server.cpp src/main.cpp ${PROGRAM_HEADERS})
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${smobex_explorer_EXPORTED_TARGETS})
