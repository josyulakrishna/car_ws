# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "velodyne_height_map: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ivelodyne_height_map:/home/cair/swahana_ws/src/velodyne_height_map/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/kinetic/share/visualization_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(velodyne_height_map_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_custom_target(_velodyne_height_map_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "velodyne_height_map" "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" "velodyne_height_map/Obstacle"
)

get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_custom_target(_velodyne_height_map_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "velodyne_height_map" "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" ""
)

get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_custom_target(_velodyne_height_map_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "velodyne_height_map" "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" "std_msgs/ColorRGBA:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Vector3:geometry_msgs/Point:visualization_msgs/MarkerArray:visualization_msgs/Marker"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_cpp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_cpp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/Marker.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map
)

### Generating Services

### Generating Module File
_generate_module_cpp(velodyne_height_map
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(velodyne_height_map_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(velodyne_height_map_generate_messages velodyne_height_map_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_cpp _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_cpp _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_cpp _velodyne_height_map_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_height_map_gencpp)
add_dependencies(velodyne_height_map_gencpp velodyne_height_map_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_height_map_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_eus(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_eus(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/Marker.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map
)

### Generating Services

### Generating Module File
_generate_module_eus(velodyne_height_map
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(velodyne_height_map_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(velodyne_height_map_generate_messages velodyne_height_map_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_eus _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_eus _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_eus _velodyne_height_map_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_height_map_geneus)
add_dependencies(velodyne_height_map_geneus velodyne_height_map_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_height_map_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_lisp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_lisp(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/Marker.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map
)

### Generating Services

### Generating Module File
_generate_module_lisp(velodyne_height_map
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(velodyne_height_map_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(velodyne_height_map_generate_messages velodyne_height_map_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_lisp _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_lisp _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_lisp _velodyne_height_map_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_height_map_genlisp)
add_dependencies(velodyne_height_map_genlisp velodyne_height_map_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_height_map_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_nodejs(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_nodejs(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/Marker.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map
)

### Generating Services

### Generating Module File
_generate_module_nodejs(velodyne_height_map
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(velodyne_height_map_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(velodyne_height_map_generate_messages velodyne_height_map_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_nodejs _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_nodejs _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_nodejs _velodyne_height_map_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_height_map_gennodejs)
add_dependencies(velodyne_height_map_gennodejs velodyne_height_map_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_height_map_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg"
  "${MSG_I_FLAGS}"
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_py(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map
)
_generate_msg_py(velodyne_height_map
  "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/ColorRGBA.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/MarkerArray.msg;/opt/ros/kinetic/share/visualization_msgs/cmake/../msg/Marker.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map
)

### Generating Services

### Generating Module File
_generate_module_py(velodyne_height_map
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(velodyne_height_map_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(velodyne_height_map_generate_messages velodyne_height_map_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacles.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_py _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Obstacle.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_py _velodyne_height_map_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/cair/swahana_ws/src/velodyne_height_map/msg/Stamped_markers_array.msg" NAME_WE)
add_dependencies(velodyne_height_map_generate_messages_py _velodyne_height_map_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(velodyne_height_map_genpy)
add_dependencies(velodyne_height_map_genpy velodyne_height_map_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS velodyne_height_map_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/velodyne_height_map
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(velodyne_height_map_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET visualization_msgs_generate_messages_cpp)
  add_dependencies(velodyne_height_map_generate_messages_cpp visualization_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/velodyne_height_map
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(velodyne_height_map_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET visualization_msgs_generate_messages_eus)
  add_dependencies(velodyne_height_map_generate_messages_eus visualization_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/velodyne_height_map
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(velodyne_height_map_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET visualization_msgs_generate_messages_lisp)
  add_dependencies(velodyne_height_map_generate_messages_lisp visualization_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/velodyne_height_map
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(velodyne_height_map_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET visualization_msgs_generate_messages_nodejs)
  add_dependencies(velodyne_height_map_generate_messages_nodejs visualization_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/velodyne_height_map
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(velodyne_height_map_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET visualization_msgs_generate_messages_py)
  add_dependencies(velodyne_height_map_generate_messages_py visualization_msgs_generate_messages_py)
endif()
