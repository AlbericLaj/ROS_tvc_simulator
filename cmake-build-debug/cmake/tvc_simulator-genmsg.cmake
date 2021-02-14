# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tvc_simulator: 5 messages, 2 services")

set(MSG_I_FLAGS "-Itvc_simulator:/home/alberic/catkin_ws/src/tvc_simulator/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tvc_simulator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" "geometry_msgs/Vector3:geometry_msgs/Point"
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" "tvc_simulator/FSM"
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" "geometry_msgs/Vector3:tvc_simulator/Waypoint:geometry_msgs/Point"
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" "geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/Twist:geometry_msgs/Vector3"
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" ""
)

get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_custom_target(_tvc_simulator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tvc_simulator" "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" "geometry_msgs/Vector3"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)

### Generating Services
_generate_srv_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv"
  "${MSG_I_FLAGS}"
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)
_generate_srv_cpp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
)

### Generating Module File
_generate_module_cpp(tvc_simulator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tvc_simulator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tvc_simulator_generate_messages tvc_simulator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_cpp _tvc_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tvc_simulator_gencpp)
add_dependencies(tvc_simulator_gencpp tvc_simulator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tvc_simulator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)
_generate_msg_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)
_generate_msg_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)
_generate_msg_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)
_generate_msg_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)

### Generating Services
_generate_srv_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv"
  "${MSG_I_FLAGS}"
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)
_generate_srv_eus(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
)

### Generating Module File
_generate_module_eus(tvc_simulator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tvc_simulator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tvc_simulator_generate_messages tvc_simulator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_eus _tvc_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tvc_simulator_geneus)
add_dependencies(tvc_simulator_geneus tvc_simulator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tvc_simulator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)
_generate_msg_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)

### Generating Services
_generate_srv_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv"
  "${MSG_I_FLAGS}"
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)
_generate_srv_lisp(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
)

### Generating Module File
_generate_module_lisp(tvc_simulator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tvc_simulator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tvc_simulator_generate_messages tvc_simulator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_lisp _tvc_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tvc_simulator_genlisp)
add_dependencies(tvc_simulator_genlisp tvc_simulator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tvc_simulator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)
_generate_msg_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)
_generate_msg_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)
_generate_msg_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)
_generate_msg_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)

### Generating Services
_generate_srv_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv"
  "${MSG_I_FLAGS}"
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)
_generate_srv_nodejs(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
)

### Generating Module File
_generate_module_nodejs(tvc_simulator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tvc_simulator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tvc_simulator_generate_messages tvc_simulator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_nodejs _tvc_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tvc_simulator_gennodejs)
add_dependencies(tvc_simulator_gennodejs tvc_simulator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tvc_simulator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)
_generate_msg_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)
_generate_msg_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)
_generate_msg_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)
_generate_msg_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)

### Generating Services
_generate_srv_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv"
  "${MSG_I_FLAGS}"
  "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)
_generate_srv_py(tvc_simulator
  "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Vector3.msg;/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
)

### Generating Module File
_generate_module_py(tvc_simulator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tvc_simulator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tvc_simulator_generate_messages tvc_simulator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Waypoint.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetFSM.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/srv/GetWaypoint.srv" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Sensor.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/State.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/FSM.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/alberic/catkin_ws/src/tvc_simulator/msg/Control.msg" NAME_WE)
add_dependencies(tvc_simulator_generate_messages_py _tvc_simulator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tvc_simulator_genpy)
add_dependencies(tvc_simulator_genpy tvc_simulator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tvc_simulator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tvc_simulator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tvc_simulator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(tvc_simulator_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tvc_simulator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tvc_simulator_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(tvc_simulator_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tvc_simulator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tvc_simulator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(tvc_simulator_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tvc_simulator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tvc_simulator_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(tvc_simulator_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tvc_simulator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tvc_simulator_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(tvc_simulator_generate_messages_py geometry_msgs_generate_messages_py)
endif()
