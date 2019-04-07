cmake_minimum_required(VERSION 3.5)
project(Wavefront)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall")

add_library(pplibrary
        src/pathplanning/WaveNav.cpp
        include/pathplanning/WaveNav.h
        src/pathplanning/OccGrid.cpp
        include/pathplanning/OccGrid.h
        src/pathplanning/Coordinates.cpp
        include/pathplanning/Coordinates.h
        )
add_executable(wavefront src/pathplanning/WaveNav.cpp)

#find_package(PkgConfig)
#if (PKG_CONFIG_FOUND)
#    pkg_check_modules(PLAYER "playerc++")
#    if (PLAYER_FOUND)
##        target_link_libraries(test ${PLAYER_LIBRARIES})
#        add_definitions(${PLAYER_CFLAGS} ${PLAYER_CFLAGS_OTHER})
#    endif ()
#endif ()

target_link_libraries(wavefront pplibrary)
#target_include_directories(wavefront PUBLIC ${PLAYER_INCLUDE_DIRS})
#target_compile_options(wavefront PUBLIC ${PLAYER_CFLAGS_OTHER})
