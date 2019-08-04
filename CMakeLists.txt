cmake_minimum_required(VERSION 3.5)
project(Wavefront)

set(CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)
#include_directories(${PROJECT_SOURCE_DIR}/lib)


#set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wextra -Werror -fsanitize=address -g -v -fstandalone-debug -O1 -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++11 -Wall -Wextra -Werror -fsanitize=address -g -v -O1 -fno-omit-frame-pointer")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++11 -Wall -Wextra -Werror -O3 -DNDEBUG")

file(GLOB SOURCES "src/pathplanning/*.cpp")


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#find_package(Boost 1.65 COMPONENTS system filesystem REQUIRED)
#
#message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
#message(STATUS "Boost_LIBRARIES: ${Boost_LIBRARIES}")
#message(STATUS "Boost_VERSION: ${Boost_VERSION}")
#include_directories(${Boost_INCLUDE_DIRS})
#link_directories(${Boost_LIBRARY_DIR})

#add_library(pplibrary
#        include/pathplanning/Coordinates.h
#        include/pathplanning/Pixel.h
#        include/pathplanning/OccGrid.h
#        include/pathplanning/DebugGrid.h
#        include/pathplanning/WaveNav.h
#        )
add_library(pplibrary SHARED ${SOURCES})

add_executable(wavefront src/pathplanning/MainDriver.cpp)
#target_link_libraries(wavefront ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})


#find_package(PkgConfig)
#if (PKG_CONFIG_FOUND)
#    pkg_check_modules(PLAYER "playerc++")
#    if (PLAYER_FOUND)
##        target_link_libraries(test ${PLAYER_LIBRARIES})
#        add_definitions(${PLAYER_CFLAGS} ${PLAYER_CFLAGS_OTHER})
#    endif ()
#endif ()

target_link_libraries(wavefront pplibrary ${OpenCV_LIBS})
#target_include_directories(wavefront PUBLIC ${PLAYER_INCLUDE_DIRS})
#target_compile_options(wavefront PUBLIC ${PLAYER_CFLAGS_OTHER})

