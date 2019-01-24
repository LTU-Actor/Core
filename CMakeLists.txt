cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_core)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
)

find_package(Lua 5.2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

add_executable(router
  src/router.cpp
)

include_directories(${catkin_INCLUDE_DIRS})

target_include_directories(router PRIVATE ${LUA_INCLUDE_DIR})
target_link_libraries(router ${catkin_LIBRARIES} ${LUA_LIBRARIES})

add_dependencies(router ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
