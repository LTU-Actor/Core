cmake_minimum_required(VERSION 3.5.0)
project(ltu_actor_core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  src/router/main.cpp
)

include_directories(${catkin_INCLUDE_DIRS})

target_include_directories(router PRIVATE ${LUA_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/external/sol2/single)
target_link_libraries(router ${catkin_LIBRARIES} ${LUA_LIBRARIES})

add_dependencies(router ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_custom_target(nextjs ALL
  COMMAND npm install
  COMMAND npm run build
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/web/"
)
