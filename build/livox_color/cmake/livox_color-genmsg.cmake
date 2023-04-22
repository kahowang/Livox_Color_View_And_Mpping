# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "livox_color: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(livox_color_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_custom_target(_livox_color_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_color" "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" ""
)

get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_custom_target(_livox_color_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "livox_color" "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_color
)
_generate_srv_cpp(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_color
)

### Generating Module File
_generate_module_cpp(livox_color
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_color
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(livox_color_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(livox_color_generate_messages livox_color_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_cpp _livox_color_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_cpp _livox_color_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_color_gencpp)
add_dependencies(livox_color_gencpp livox_color_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_color_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_color
)
_generate_srv_eus(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_color
)

### Generating Module File
_generate_module_eus(livox_color
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_color
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(livox_color_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(livox_color_generate_messages livox_color_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_eus _livox_color_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_eus _livox_color_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_color_geneus)
add_dependencies(livox_color_geneus livox_color_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_color_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_color
)
_generate_srv_lisp(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_color
)

### Generating Module File
_generate_module_lisp(livox_color
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_color
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(livox_color_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(livox_color_generate_messages livox_color_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_lisp _livox_color_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_lisp _livox_color_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_color_genlisp)
add_dependencies(livox_color_genlisp livox_color_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_color_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_color
)
_generate_srv_nodejs(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_color
)

### Generating Module File
_generate_module_nodejs(livox_color
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_color
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(livox_color_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(livox_color_generate_messages livox_color_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_nodejs _livox_color_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_nodejs _livox_color_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_color_gennodejs)
add_dependencies(livox_color_gennodejs livox_color_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_color_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color
)
_generate_srv_py(livox_color
  "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color
)

### Generating Module File
_generate_module_py(livox_color
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(livox_color_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(livox_color_generate_messages livox_color_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_map.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_py _livox_color_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lory/livox_color_ws/src/livox_color/srv/save_pose.srv" NAME_WE)
add_dependencies(livox_color_generate_messages_py _livox_color_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(livox_color_genpy)
add_dependencies(livox_color_genpy livox_color_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS livox_color_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_color)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/livox_color
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(livox_color_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_color)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/livox_color
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(livox_color_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_color)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/livox_color
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(livox_color_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_color)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/livox_color
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(livox_color_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/livox_color
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(livox_color_generate_messages_py geometry_msgs_generate_messages_py)
endif()
