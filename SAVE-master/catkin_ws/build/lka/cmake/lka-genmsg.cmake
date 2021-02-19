# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "lka: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ilka:C:/catkin_ws/src/lka/msg;-Istd_msgs:C:/opt/ros/melodic/x64/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(lka_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_custom_target(_lka_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lka" "C:/catkin_ws/src/lka/msg/Lane.msg" ""
)

get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_custom_target(_lka_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lka" "C:/catkin_ws/src/lka/msg/Margins.msg" ""
)

get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_custom_target(_lka_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "lka" "C:/catkin_ws/src/lka/msg/Lanes.msg" "lka/Lane"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(lka
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka
)
_generate_msg_cpp(lka
  "C:/catkin_ws/src/lka/msg/Margins.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka
)
_generate_msg_cpp(lka
  "C:/catkin_ws/src/lka/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka
)

### Generating Services

### Generating Module File
_generate_module_cpp(lka
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(lka_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(lka_generate_messages lka_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_dependencies(lka_generate_messages_cpp _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_dependencies(lka_generate_messages_cpp _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_dependencies(lka_generate_messages_cpp _lka_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lka_gencpp)
add_dependencies(lka_gencpp lka_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lka_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(lka
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka
)
_generate_msg_eus(lka
  "C:/catkin_ws/src/lka/msg/Margins.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka
)
_generate_msg_eus(lka
  "C:/catkin_ws/src/lka/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka
)

### Generating Services

### Generating Module File
_generate_module_eus(lka
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(lka_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(lka_generate_messages lka_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_dependencies(lka_generate_messages_eus _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_dependencies(lka_generate_messages_eus _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_dependencies(lka_generate_messages_eus _lka_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lka_geneus)
add_dependencies(lka_geneus lka_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lka_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(lka
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka
)
_generate_msg_lisp(lka
  "C:/catkin_ws/src/lka/msg/Margins.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka
)
_generate_msg_lisp(lka
  "C:/catkin_ws/src/lka/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka
)

### Generating Services

### Generating Module File
_generate_module_lisp(lka
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(lka_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(lka_generate_messages lka_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_dependencies(lka_generate_messages_lisp _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_dependencies(lka_generate_messages_lisp _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_dependencies(lka_generate_messages_lisp _lka_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lka_genlisp)
add_dependencies(lka_genlisp lka_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lka_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(lka
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka
)
_generate_msg_nodejs(lka
  "C:/catkin_ws/src/lka/msg/Margins.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka
)
_generate_msg_nodejs(lka
  "C:/catkin_ws/src/lka/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka
)

### Generating Services

### Generating Module File
_generate_module_nodejs(lka
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(lka_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(lka_generate_messages lka_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_dependencies(lka_generate_messages_nodejs _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_dependencies(lka_generate_messages_nodejs _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_dependencies(lka_generate_messages_nodejs _lka_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lka_gennodejs)
add_dependencies(lka_gennodejs lka_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lka_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(lka
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka
)
_generate_msg_py(lka
  "C:/catkin_ws/src/lka/msg/Margins.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka
)
_generate_msg_py(lka
  "C:/catkin_ws/src/lka/msg/Lanes.msg"
  "${MSG_I_FLAGS}"
  "C:/catkin_ws/src/lka/msg/Lane.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka
)

### Generating Services

### Generating Module File
_generate_module_py(lka
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(lka_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(lka_generate_messages lka_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lane.msg" NAME_WE)
add_dependencies(lka_generate_messages_py _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Margins.msg" NAME_WE)
add_dependencies(lka_generate_messages_py _lka_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/catkin_ws/src/lka/msg/Lanes.msg" NAME_WE)
add_dependencies(lka_generate_messages_py _lka_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(lka_genpy)
add_dependencies(lka_genpy lka_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS lka_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/lka
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(lka_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/lka
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(lka_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/lka
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(lka_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/lka
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(lka_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka)
  install(CODE "execute_process(COMMAND \"C:/opt/ros/melodic/x64/python.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/lka
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(lka_generate_messages_py std_msgs_generate_messages_py)
endif()
