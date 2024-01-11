#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "project_name::turtlelib" for configuration ""
set_property(TARGET project_name::turtlelib APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(project_name::turtlelib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libturtlelib.a"
  )

list(APPEND _cmake_import_check_targets project_name::turtlelib )
list(APPEND _cmake_import_check_files_for_project_name::turtlelib "${_IMPORT_PREFIX}/lib/libturtlelib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
