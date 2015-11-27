cmake_minimum_required(VERSION 2.8.0)

project(Video)

set(HEADER_FILES
    IVideoCapture.h
    IVideoPlayer.h
    VideoCapture.h
    VideoPlayback.h
    VideoPlayer.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    VideoCapture.cpp
    VideoPlayback.cpp
    VideoPlayer.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time random iostreams thread system)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_library(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)
