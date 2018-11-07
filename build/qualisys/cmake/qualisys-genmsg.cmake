# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qualisys: 3 messages, 0 services")

set(MSG_I_FLAGS "-Iqualisys:/home/nvidia/catkin_ws/src/qualisys/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qualisys_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_custom_target(_qualisys_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qualisys" "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" "qualisys/Marker:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_custom_target(_qualisys_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qualisys" "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_custom_target(_qualisys_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qualisys" "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" "geometry_msgs/Quaternion:qualisys/Marker:std_msgs/Header:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys
)
_generate_msg_cpp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys
)
_generate_msg_cpp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys
)

### Generating Services

### Generating Module File
_generate_module_cpp(qualisys
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qualisys_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qualisys_generate_messages qualisys_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_cpp _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_cpp _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_cpp _qualisys_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qualisys_gencpp)
add_dependencies(qualisys_gencpp qualisys_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qualisys_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys
)
_generate_msg_eus(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys
)
_generate_msg_eus(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys
)

### Generating Services

### Generating Module File
_generate_module_eus(qualisys
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qualisys_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qualisys_generate_messages qualisys_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_eus _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_eus _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_eus _qualisys_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qualisys_geneus)
add_dependencies(qualisys_geneus qualisys_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qualisys_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys
)
_generate_msg_lisp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys
)
_generate_msg_lisp(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys
)

### Generating Services

### Generating Module File
_generate_module_lisp(qualisys
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qualisys_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qualisys_generate_messages qualisys_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_lisp _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_lisp _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_lisp _qualisys_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qualisys_genlisp)
add_dependencies(qualisys_genlisp qualisys_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qualisys_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys
)
_generate_msg_nodejs(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys
)
_generate_msg_nodejs(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys
)

### Generating Services

### Generating Module File
_generate_module_nodejs(qualisys
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qualisys_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qualisys_generate_messages qualisys_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_nodejs _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_nodejs _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_nodejs _qualisys_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qualisys_gennodejs)
add_dependencies(qualisys_gennodejs qualisys_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qualisys_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg"
  "${MSG_I_FLAGS}"
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys
)
_generate_msg_py(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys
)
_generate_msg_py(qualisys
  "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys
)

### Generating Services

### Generating Module File
_generate_module_py(qualisys
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qualisys_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qualisys_generate_messages qualisys_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Markers.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_py _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Marker.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_py _qualisys_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/nvidia/catkin_ws/src/qualisys/msg/Subject.msg" NAME_WE)
add_dependencies(qualisys_generate_messages_py _qualisys_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qualisys_genpy)
add_dependencies(qualisys_genpy qualisys_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qualisys_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qualisys
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(qualisys_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qualisys
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(qualisys_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qualisys
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(qualisys_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qualisys
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(qualisys_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qualisys
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(qualisys_generate_messages_py geometry_msgs_generate_messages_py)
endif()
