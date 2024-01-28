# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
# find the c and c++ compilers
project(turtlelib)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(turtlelib src/geometry2d.cpp src/se2d.cpp src/svg.cpp src/diff_drive.cpp)

target_include_directories(turtlelib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

target_compile_options(turtlelib PUBLIC -Wall -Wextra -pedantic)

target_compile_features(turtlelib PUBLIC cxx_std_17)

add_executable(frame_main src/frame_main.cpp)

target_link_libraries(frame_main turtlelib)

# install the include files by copying the whole include directory
install(DIRECTORY include DESTINATION include)

install(TARGETS ${PROJECT_NAME} EXPORT project_name-targets)

install(EXPORT project_name-targets
        FILE project_name-config.cmake
        NAMESPACE project_name::
        DESTINATION lib/cmake/${PROJECT_NAME})

find_package(Doxygen)       

option(BUILD_DOCS "Build the documentation" ON)

if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

endif()

include(CTest)

# CTest sets BUILD_TESTING to on. To disable tests add -DBUILD_TESTING=OFF when invoking cmake
if(BUILD_TESTING)
    # Find the Unit testing framework. In this example, Catch2
    find_package(Catch2 3 REQUIRED)

    # A test is just an executable that is linked against the unit testing library
    add_executable (my_test_exe tests/test_geometry2d.cpp tests/test_se2d.cpp tests/test_svg.cpp tests/test_diff_drive.cpp)

    target_link_libraries(my_test_exe Catch2::Catch2WithMain turtlelib)

    target_include_directories(my_test_exe
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

    add_test(NAME test_name COMMAND my_test_exe)
endif()