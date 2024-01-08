#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "coverage_contest::game_visualizer" for configuration ""
set_property(TARGET coverage_contest::game_visualizer APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(coverage_contest::game_visualizer PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgame_visualizer.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS coverage_contest::game_visualizer )
list(APPEND _IMPORT_CHECK_FILES_FOR_coverage_contest::game_visualizer "${_IMPORT_PREFIX}/lib/libgame_visualizer.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
