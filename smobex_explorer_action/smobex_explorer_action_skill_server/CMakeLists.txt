cmake_minimum_required(VERSION 2.8.3)
project(smobex_explorer_action_skill_server)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  smobex_explorer_action_skill_msgs
)

file(GLOB PROGRAM_HEADERS RELATIVE ${PROJECT_SOURCE_DIR} "include/${PROJECT_NAME}/*.h")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(${PROJECT_NAME} src/smobex_explorer_action_skill_server.cpp src/main.cpp ${PROGRAM_HEADERS})
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
