cmake_minimum_required(VERSION 3.5)
project(Wavefront)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

add_library(proj4lib
        lib/WavefrontNavigator.cpp
        lib/WavefrontNavigator.h
        lib/OccGrid.cpp
        lib/OccGrid.h
        lib/GridCell.cpp
        lib/GridCell.h
        lib/Point.cpp
        lib/Point.h)

add_executable(test lib/WavefrontNavigator.cpp)
# add_executable(proj4 src/proj4.cc)

find_package(PkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(PLAYER "playerc++")
    if (PLAYER_FOUND)
        target_link_libraries(test ${PLAYER_LIBRARIES})
        add_definitions(${PLAYER_CFLAGS} ${PLAYER_CFLAGS_OTHER})
    endif ()
endif ()

target_link_libraries(test proj4lib)
target_include_directories(test PUBLIC ${PLAYER_INCLUDE_DIRS})
target_compile_options(test PUBLIC ${PLAYER_CFLAGS_OTHER})
