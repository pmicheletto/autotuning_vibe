#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "fmi4cpp::fmi4cpp" for configuration "Release"
set_property(TARGET fmi4cpp::fmi4cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(fmi4cpp::fmi4cpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libfmi4cpp.so"
  IMPORTED_SONAME_RELEASE "libfmi4cpp.so"
  )

list(APPEND _cmake_import_check_targets fmi4cpp::fmi4cpp )
list(APPEND _cmake_import_check_files_for_fmi4cpp::fmi4cpp "${_IMPORT_PREFIX}/lib/libfmi4cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
