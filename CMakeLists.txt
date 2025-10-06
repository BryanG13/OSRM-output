cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Only set default build type if none specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
cmake_policy(SET CMP0144 NEW)
cmake_policy(SET CMP0167 NEW)

# Only enable AddressSanitizer for Debug builds --> this helps finding memory issues 
if(CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -g")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
endif()

# Project name
project(osrm)

# Jsoncpp
set(JSONCPP_LIBRARIES "/opt/homebrew/Cellar/jsoncpp/1.9.6/lib/libjsoncpp.dylib")
link_libraries(${JSONCPP_LIBRARIES})
include_directories("/opt/homebrew/Cellar/jsoncpp/1.9.6/include")

# OSRM
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(LibOSRM REQUIRED)
link_directories(${LibOSRM_LIBRARY_DIRS})
include_directories(SYSTEM ${LibOSRM_INCLUDE_DIRS})

# Threads 
find_package(Threads REQUIRED)

# Include Boost
set(BOOST_ROOT "/opt/homebrew/Cellar/boost/1.89.0")

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options date_time iostreams thread)
if (Boost_FOUND)
    message(STATUS "Boost found: ${Boost_VERSION}")
    message(STATUS "Boost include dirs: ${Boost_INCLUDE_DIRS}")
    message(STATUS "Boost libraries: ${Boost_LIBRARIES}")
else()
    message(FATAL_ERROR "Boost not found!")
endif()

link_libraries(${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

# Add all .cpp files in src folder to the executable
file(GLOB_RECURSE SRC_FILES "src/*.cpp")

add_executable(osrm ${SRC_FILES})

# Link libraries to executable 

target_link_libraries(osrm ${Boost_LIBRARIES})
target_link_libraries(osrm ${JSONCPP_LIBRARIES})
target_link_libraries(osrm ${LibOSRM_LIBRARIES} ${LibOSRM_DEPENDENT_LIBRARIES})
target_link_libraries(osrm Threads::Threads)


# Set compiler flags 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LibOSRM_CXXFLAGS}")

# For inellisense config 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)



# Enable verbose linking
#set(CMAKE_VERBOSE_MAKEFILE ON)