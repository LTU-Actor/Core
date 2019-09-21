cmake_minimum_required(VERSION 3.5.0)
project(ltu_actor_core)
add_definitions(--std=c++17)


find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation
)

add_service_files(
  FILES
    GetCurrentRoute.srv
    GetRouteList.srv
    LoadRoute.srv
    SaveRoute.srv
    SetTemporaryRoute.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

find_package(Lua 5.2 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    geometry_msgs
    message_runtime
)

add_executable(router
  src/router/main.cpp
  src/router/subpub.h
  src/router/setup.lua
  src/router/default_route.lua
)

include_directories(${catkin_INCLUDE_DIRS})

target_include_directories(router PRIVATE ${LUA_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/external/sol2/single)
target_link_libraries(router ${catkin_LIBRARIES} ${LUA_LIBRARIES})
add_dependencies(router ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
