cmake_minimum_required(VERSION 3.0.2)
project(relaxed_astar_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  nav_core
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES relaxed_astar_planner
   CATKIN_DEPENDS 
    pluginlib
    roscpp
    nav_core
   DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(relaxed_astar_planner src/relaxed_astar_planner.cpp)
target_link_libraries(relaxed_astar_planner ${catkin_LIBRARIES})
add_dependencies(relaxed_astar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS relaxed_astar_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
