# Option to enable code coverage.
option(CODE_COVERAGE "Enable coverage reporting" ON)
if(CODE_COVERAGE)
  # Use coverage flags if using GCC or Clang.
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "Building with coverage flags")
    # Disable optimizations (-O0) to have a more accurate coverage report.
    set(COVERAGE_COMPILE_FLAGS "-O0 -g --coverage -fprofile-arcs -ftest-coverage")
    set(COVERAGE_LINK_FLAGS    "--coverage -fprofile-arcs -ftest-coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
  else()
    message(WARNING "Code coverage is not supported for this compiler")
  endif()
endif()

# find gtest and gmock
pkg_check_modules(GTest REQUIRED gtest)
pkg_check_modules(GMock REQUIRED gmock)
include_directories(${GTest_INCLUDE_DIRS} ${GMock_INCLUDE_DIRS})
link_directories(${GTest_LIBRARY_DIRS} ${GMock_LIBRARY_DIRS})


# test executables
add_executable(test_gnc test_gnc.cpp ../src/gnc.cpp)
add_executable(test_physics test_physics.cpp)
add_executable(test_propulsion test_propulsion.cpp ../src/propulsion.cpp)

# link test executables to GTest
target_link_libraries(test_gnc ${GTest_LIBRARIES} gtest_main gmock_main)
target_link_libraries(test_physics ${GTest_LIBRARIES} gtest_main gmock_main)
target_link_libraries(test_propulsion ${GTest_LIBRARIES} gtest_main gmock_main)

# add tests to ctest
add_test(NAME test_gnc COMMAND test_gnc)
add_test(NAME test_physics COMMAND test_physics)
add_test(NAME test_propulsion COMMAND test_propulsion)

# Find lcov and genhtml (required for generating the gcov report).
find_program(LCOV_EXEC lcov)
find_program(GENHTML_EXEC genhtml)

# If lcov and genhtml are found, create a custom target for coverage.
if(LCOV_EXEC AND GENHTML_EXEC)
  # This custom target runs tests and then generates an HTML coverage report.
  add_custom_target(coverage
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMAND ${LCOV_EXEC} --directory . --capture --output-file coverage.info --ignore-errors inconsistent
    # Optionally remove system files from coverage (e.g., files from /usr).
    COMMAND ${LCOV_EXEC} --remove coverage.info "/usr/*" --output-file coverage.info --ignore-errors unused --ignore-errors inconsistent --rc geninfo_unexecuted_blocks=1
    COMMAND ${GENHTML_EXEC} coverage.info --output-directory coverage_report
    COMMENT "Running tests and generating code coverage report in the coverage_report directory"
  )
else()
  message(WARNING "lcov and/or genhtml not found! The \"coverage\" target will not be available.")
endif()