# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "controls: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icontrols:C:/catkin_ws/src/controls/msg;-Istd_msgs:C:/opt/ros/melodic/x64/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(controls_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_custom_target(_controls_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "controls" "C:/catkin_ws/src/controls/msg/Control.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(controls
  "C:/catkin_ws/src/controls/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controls
)

### Generating Services

### Generating Module File
_generate_module_cpp(controls
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controls
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(controls_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(controls_generate_messages controls_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_dependencies(controls_generate_messages_cpp _controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controls_gencpp)
add_dependencies(controls_gencpp controls_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controls_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(controls
  "C:/catkin_ws/src/controls/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controls
)

### Generating Services

### Generating Module File
_generate_module_eus(controls
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controls
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(controls_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(controls_generate_messages controls_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_dependencies(controls_generate_messages_eus _controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controls_geneus)
add_dependencies(controls_geneus controls_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controls_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(controls
  "C:/catkin_ws/src/controls/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controls
)

### Generating Services

### Generating Module File
_generate_module_lisp(controls
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controls
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(controls_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(controls_generate_messages controls_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_dependencies(controls_generate_messages_lisp _controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controls_genlisp)
add_dependencies(controls_genlisp controls_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controls_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(controls
  "C:/catkin_ws/src/controls/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controls
)

### Generating Services

### Generating Module File
_generate_module_nodejs(controls
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controls
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(controls_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(controls_generate_messages controls_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_dependencies(controls_generate_messages_nodejs _controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controls_gennodejs)
add_dependencies(controls_gennodejs controls_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controls_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(controls
  "C:/catkin_ws/src/controls/msg/Control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controls
)

### Generating Services

### Generating Module File
_generate_module_py(controls
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controls
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(controls_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(controls_generate_messages controls_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/controls/msg/Control.msg" NAME_WE)
add_dependencies(controls_generate_messages_py _controls_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(controls_genpy)
add_dependencies(controls_genpy controls_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS controls_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/controls
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(controls_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/controls
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(controls_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/controls
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(controls_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controls)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/controls
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(controls_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controls)
  install(CODE "execute_process(COMMAND \"C:/opt/ros/melodic/x64/python.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controls\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/controls
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(controls_generate_messages_py std_msgs_generate_messages_py)
endif()
