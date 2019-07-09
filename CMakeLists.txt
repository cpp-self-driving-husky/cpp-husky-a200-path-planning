cmake_minimum_required(VERSION 3.5)
project(Wavefront)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Werror -fsanitize=address -g -v -fstandalone-debug -O0")


add_library(pplibrary
        include/pathplanning/Coordinates.h
        src/pathplanning/Coordinates.cpp
        include/pathplanning/OccGrid.h
        src/pathplanning/OccGrid.cpp
        include/pathplanning/WaveNav.h
        src/pathplanning/WaveNav.cpp
        include/pathplanning/DebugGrid.h
        include/pathplanning/Pixel.h
        src/pathplanning/Pixel.cpp
        src/pathplanning/DebugGrid.cpp)

find_package(Boost 1.65 COMPONENTS system filesystem REQUIRED)

message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
message(STATUS "Boost_VERSION: ${Boost_VERSION}")
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIR})

add_executable(wavefront src/pathplanning/WaveNav.cpp)
target_link_libraries(wavefront ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})


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

