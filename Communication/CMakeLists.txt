cmake_minimum_required(VERSION 2.8.0)

project(Communication)

set(HEADER_FILES
    CommunicationData.h
    Connection.h
    DataGram.h
    DataGramPack.h
    FrameStream.h
    PerformanceTimer.h
    StreamControl.h
    Statistics.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    Connection.cpp
    DataGramPack.cpp
    StreamControl.cpp
    DataGram.cpp
    FrameStream.cpp
    PerformanceTimer.cpp
    Statistics.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem date_time iostreams thread system)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Utils_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
)

add_library(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)

