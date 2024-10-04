# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ukf_ais: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iukf_ais:/home/phlyoo/catkin_ws/src/ukf_ais/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ukf_ais_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_custom_target(_ukf_ais_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ukf_ais" "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" ""
)

get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_custom_target(_ukf_ais_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ukf_ais" "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ukf_ais
)
_generate_msg_cpp(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ukf_ais
)

### Generating Services

### Generating Module File
_generate_module_cpp(ukf_ais
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ukf_ais
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ukf_ais_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ukf_ais_generate_messages ukf_ais_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_cpp _ukf_ais_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_cpp _ukf_ais_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ukf_ais_gencpp)
add_dependencies(ukf_ais_gencpp ukf_ais_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ukf_ais_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ukf_ais
)
_generate_msg_eus(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ukf_ais
)

### Generating Services

### Generating Module File
_generate_module_eus(ukf_ais
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ukf_ais
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ukf_ais_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ukf_ais_generate_messages ukf_ais_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_eus _ukf_ais_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_eus _ukf_ais_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ukf_ais_geneus)
add_dependencies(ukf_ais_geneus ukf_ais_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ukf_ais_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ukf_ais
)
_generate_msg_lisp(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ukf_ais
)

### Generating Services

### Generating Module File
_generate_module_lisp(ukf_ais
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ukf_ais
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ukf_ais_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ukf_ais_generate_messages ukf_ais_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_lisp _ukf_ais_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_lisp _ukf_ais_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ukf_ais_genlisp)
add_dependencies(ukf_ais_genlisp ukf_ais_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ukf_ais_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ukf_ais
)
_generate_msg_nodejs(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ukf_ais
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ukf_ais
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ukf_ais
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ukf_ais_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ukf_ais_generate_messages ukf_ais_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_nodejs _ukf_ais_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_nodejs _ukf_ais_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ukf_ais_gennodejs)
add_dependencies(ukf_ais_gennodejs ukf_ais_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ukf_ais_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais
)
_generate_msg_py(ukf_ais
  "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais
)

### Generating Services

### Generating Module File
_generate_module_py(ukf_ais
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ukf_ais_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ukf_ais_generate_messages ukf_ais_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ResultInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_py _ukf_ais_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/phlyoo/catkin_ws/src/ukf_ais/msg/ShipInfo.msg" NAME_WE)
add_dependencies(ukf_ais_generate_messages_py _ukf_ais_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ukf_ais_genpy)
add_dependencies(ukf_ais_genpy ukf_ais_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ukf_ais_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ukf_ais)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ukf_ais
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ukf_ais_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ukf_ais)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ukf_ais
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ukf_ais_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ukf_ais)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ukf_ais
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ukf_ais_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ukf_ais)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ukf_ais
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ukf_ais_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ukf_ais
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ukf_ais_generate_messages_py std_msgs_generate_messages_py)
endif()
