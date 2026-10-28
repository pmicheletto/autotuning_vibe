# Self-test for the bundled tinyzip / pugixml-lite implementations: extracts
# an FMU (every entry CRC-verified) and parses its modelDescription.xml.
# The sources themselves are compiled into the fmi4cpp target (see
# ../src/CMakeLists.txt).

add_executable(fmi4cpp_third_party_selftest
    selftest.cpp
    tinyzip/tinyzip.cpp
    pugixml-lite/pugixml.cpp
)
target_include_directories(fmi4cpp_third_party_selftest PRIVATE tinyzip pugixml-lite)
target_compile_features(fmi4cpp_third_party_selftest PRIVATE cxx_std_17)
add_test(
    NAME fmi4cpp_third_party_selftest
    COMMAND fmi4cpp_third_party_selftest
        "${PROJECT_SOURCE_DIR}/resources/fmus/2.0/cs/20sim/4.6.4.8004/ControlledTemperature/ControlledTemperature.fmu"
)
