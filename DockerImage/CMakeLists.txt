cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)
cmake_policy(SET CMP0144 NEW)

# Project name
project(osrm)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(PkgConfig REQUIRED)

# Jsoncpp
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
link_libraries(${JSONCPP_LIBRARIES})

# OSRM
find_package(LibOSRM REQUIRED)
link_directories(${LibOSRM_LIBRARY_DIRS})
include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})

# Boost
find_package(Boost REQUIRED COMPONENTS program_options date_time iostreams thread)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})


include_directories(${PROJECT_SOURCE_DIR}/include)

# Add all .cpp files in src folder to the executable
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_executable(osrm ${SRC_FILES})

# Link libraries to executable 

target_link_libraries(osrm ${Boost_LIBRARIES})
target_link_libraries(osrm ${JSONCPP_LIBRARIES})
target_link_libraries(osrm ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})

# Set compiler flags 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")


# Enable verbose linking
#set(CMAKE_VERBOSE_MAKEFILE ON)