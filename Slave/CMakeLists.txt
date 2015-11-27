cmake_minimum_required(VERSION 2.8.0)

project(Slave)

set(HEADER_FILES
    Slave.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    Slave.cpp
    main.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Communication_SOURCE_DIR}
    ${Video_SOURCE_DIR}
    ${Utils_SOURCE_DIR}
    ${LIBXML2_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
)

add_executable(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

if (${YOUBOT} MATCHES "ON")

    include_directories(
        ${YouBot_SOURCE_DIR}
        ${YOUBOT_INCLUDE}
        ${ETHERCAT_INCLUDE}
    )

    target_link_libraries(
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OPENCV_HIGHGUI}
        ${LIBXML2_LIBRARIES}
        ${YOUBOT_DRIVER}
        ${LIBSOEM}
        Communication
        Video
        Utils
        YouBot
    )
else()

    target_link_libraries(
        ${PROJECT_NAME}
        ${Boost_LIBRARIES}
        ${CMAKE_THREAD_LIBS_INIT}
        ${OpenCV_LIBS}
        ${LIBXML2_LIBRARIES}
        Communication
        Video
        Utils
    )
endif()
