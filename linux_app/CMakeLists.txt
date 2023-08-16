cmake_minimum_required(VERSION 2.8.3)
project(ldlidar_stl)

find_package(OpenCV REQUIRED)
find_package(OpenAL REQUIRED)

## Compile option
if(${CMAKE_BUILD_TYPE} MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -Wall")
  #set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -Wall")
  message(STATUS "Mode: Release")
  message(STATUS "optional:-std=c++11 -Wall")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
else()
  set(CMAKE_BUILD_TYPE "Debug") 
  set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
  #set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O2 -Wall -Wextra -Wpedantic -g2 -ggdb")
  message(STATUS "Mode: Debug")
  message(STATUS "optional:-std=c++11 -Wall -Wextra -Wpedantic -g2 -ggdb")
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/core/
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/dataprocess/
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/filter/
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/logger/
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/networkcom/
${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/include/serialcom/
${CMAKE_CURRENT_SOURCE_DIR}/src/include/
${OpenCV_INCLUDE_DIRS}
${OpenAL_INCLUDE_DIRS}
)

file(GLOB  LDLIDAR_DRI_CORE ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/core/*.cpp)
file(GLOB  LDLIDAR_DRI_DATARPC ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/dataprocess/*.cpp)
file(GLOB  LDLIDAR_DRI_FILTER ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/filter/*.cpp)
file(GLOB  LDLIDAR_DRI_LOGGER ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/logger/*.cpp)
file(GLOB  LDLIDAR_DRI_NETWORK ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/networkcom/*.cpp)
file(GLOB  LDLIDAR_DRI_SERIAL ${CMAKE_CURRENT_SOURCE_DIR}/../ldlidar_driver/src/serialcom/*.cpp)

file(GLOB  SHOW_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/demo/show_demo/*.cpp)
file(GLOB  MATCH_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/demo/match_demo/*.cpp)
file(GLOB  VOICE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/demo/voice_demo/*.cpp)
file(GLOB  PRESSURE_TEST ${CMAKE_CURRENT_SOURCE_DIR}/src/pressure_test/*.cpp)

add_executable(show_demo 
${SHOW_SRC} 
${LDLIDAR_DRI_CORE} 
${LDLIDAR_DRI_DATARPC} 
${LDLIDAR_DRI_FILTER} 
${LDLIDAR_DRI_LOGGER} 
${LDLIDAR_DRI_NETWORK} 
${LDLIDAR_DRI_SERIAL})
target_link_libraries(show_demo pthread)
target_link_libraries(show_demo ${OpenCV_LIBS})

add_executable(match_demo 
${MATCH_SRC} 
${LDLIDAR_DRI_CORE} 
${LDLIDAR_DRI_DATARPC} 
${LDLIDAR_DRI_FILTER} 
${LDLIDAR_DRI_LOGGER} 
${LDLIDAR_DRI_NETWORK} 
${LDLIDAR_DRI_SERIAL})
target_link_libraries(match_demo pthread)
target_link_libraries(match_demo ${OpenCV_LIBS})

add_executable(voice_demo 
${VOICE_SRC} 
${LDLIDAR_DRI_CORE} 
${LDLIDAR_DRI_DATARPC} 
${LDLIDAR_DRI_FILTER} 
${LDLIDAR_DRI_LOGGER} 
${LDLIDAR_DRI_NETWORK} 
${LDLIDAR_DRI_SERIAL})
target_link_libraries(voice_demo pthread)
target_link_libraries(voice_demo ${OpenCV_LIBS})
target_link_libraries(voice_demo openal)

add_executable(pressure_test 
${PRESSURE_TEST} 
${LDLIDAR_DRI_CORE} 
${LDLIDAR_DRI_DATARPC} 
${LDLIDAR_DRI_FILTER} 
${LDLIDAR_DRI_LOGGER} 
${LDLIDAR_DRI_NETWORK} 
${LDLIDAR_DRI_SERIAL})
target_link_libraries(pressure_test pthread)

