# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(timesync_CONFIG_INCLUDED)
  return()
endif()
set(timesync_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(timesync_SOURCE_PREFIX /home/seldat/seldat_robot/src/timesync_ros)
  set(timesync_DEVEL_PREFIX /home/seldat/seldat_robot/devel)
  set(timesync_INSTALL_PREFIX "")
  set(timesync_PREFIX ${timesync_DEVEL_PREFIX})
else()
  set(timesync_SOURCE_PREFIX "")
  set(timesync_DEVEL_PREFIX "")
  set(timesync_INSTALL_PREFIX /home/seldat/seldat_robot/install)
  set(timesync_PREFIX ${timesync_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'timesync' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(timesync_FOUND_CATKIN_PROJECT TRUE)

if(NOT "/home/seldat/seldat_robot/devel/include;/home/seldat/seldat_robot/src/timesync_ros/include " STREQUAL " ")
  set(timesync_INCLUDE_DIRS "")
  set(_include_dirs "/home/seldat/seldat_robot/devel/include;/home/seldat/seldat_robot/src/timesync_ros/include")
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${timesync_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'timesync' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  Ask the maintainer 'Juraj Oršulić <juraj.orsulic@fer.hr>' to fix it.")
      endif()
    else()
      message(FATAL_ERROR "Project 'timesync' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/seldat/seldat_robot/src/timesync_ros/${idir}'.  Ask the maintainer 'Juraj Oršulić <juraj.orsulic@fer.hr>' to fix it.")
    endif()
    _list_append_unique(timesync_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "timesync")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND timesync_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND timesync_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND timesync_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/seldat/seldat_robot/devel/lib;/opt/ros/indigo/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(timesync_LIBRARY_DIRS ${lib_path})
      list(APPEND timesync_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'timesync'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND timesync_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(timesync_EXPORTED_TARGETS "timesync_generate_messages_cpp;timesync_generate_messages_lisp;timesync_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${timesync_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "dynamic_reconfigure;roscpp;message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 timesync_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${timesync_dep}_FOUND)
      find_package(${timesync_dep} REQUIRED)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${timesync_dep} REQUIRED ${depend_list})
  endif()
  _list_append_unique(timesync_INCLUDE_DIRS ${${timesync_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(timesync_LIBRARIES ${timesync_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${timesync_dep}_LIBRARIES})
  _list_append_deduplicate(timesync_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(timesync_LIBRARIES ${timesync_LIBRARIES})

  _list_append_unique(timesync_LIBRARY_DIRS ${${timesync_dep}_LIBRARY_DIRS})
  list(APPEND timesync_EXPORTED_TARGETS ${${timesync_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "timesync-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${timesync_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
