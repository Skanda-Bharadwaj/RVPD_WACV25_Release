cmake_minimum_required( VERSION 2.8 )
project (RVPD_WACV25)

message("${CMAKE_MODULE_PATH}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

############### dependencies ######################

# OpenCV

set(OpenCV_DIR "/home/skanda/Softwares/OpenCV/opencv/build")
find_package(OpenCV REQUIRED)
# message(status "** OpenCV Libraries: ${OpenCV_LIBRARIES}")

# Boost
# set(BOOST_ROOT "D:/boost_1_74_0")
# set(BOOST_LIBRARYDIR "D:/boost_1_74_0/stage/lib") 

# set(Boost_USE_STATIC_LIBS ON)
# et(Boost_USE_MULTITHREAD ON)

find_package(Boost COMPONENTS filesystem program_options REQUIRED)
include_directories( ${Boost_INCLUDE_DIRS} )
message(status "** Boost Libraries: ${Boost_LIBRARIES}")

include_directories( "/usr/include/eigen3" )

set( THIRD_PARTY_LIBS
${OpenCV_LIBRARIES}
${Boost_LIBRARIES}
)


# Source and Header
file(GLOB_RECURSE SRC_CORE Source/*.cpp)
file(GLOB_RECURSE HEADER_CORE Header/*.h)

############### Build the whole project ######################
add_executable(${PROJECT_NAME} ${SRC_CORE} ${HEADER_CORE})

target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/Header )
target_link_libraries( ${PROJECT_NAME} ${THIRD_PARTY_LIBS})
