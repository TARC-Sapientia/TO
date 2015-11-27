cmake_minimum_required(VERSION 2.8.0)
project(Master)

set(HEADER_FILES
    Master.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    Master.cpp
    main.cpp
)

source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(LibXml2 REQUIRED)
find_package(OpenCV REQUIRED)

if (${HAPTIC} MATCHES "OFF")
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
    ${Communication_SOURCE_DIR}
    ${Video_SOURCE_DIR}
    ${Utils_SOURCE_DIR}
    ${MasterController_SOURCE_DIR}
)
endif()

if (${HAPTIC} MATCHES "PHANTOM_OMNI")
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
    ${Communication_SOURCE_DIR}
    ${Video_SOURCE_DIR}
    ${Utils_SOURCE_DIR}
    ${MasterController_SOURCE_DIR}
    ${HapticPhantomOmni_SOURCE_DIR}
)
endif()

if (${HAPTIC} MATCHES "NOVINT_FALCON")
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
    ${Communication_SOURCE_DIR}
    ${Video_SOURCE_DIR}
    ${Utils_SOURCE_DIR}
    ${MasterController_SOURCE_DIR}
    ${HapticNovintFalcon_SOURCE_DIR}
)
endif()

add_executable(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

if (${HAPTIC} MATCHES "OFF")
target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${LIBXML2_LIBRARIES}
    Communication
    Video
    Utils
    MasterController
)
endif()

if (${HAPTIC} MATCHES "PHANTOM_OMNI")
target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${LIBXML2_LIBRARIES}
    Communication
    Video
    Utils
    MasterController
    HapticPhantomOmni
)
endif()

if (${HAPTIC} MATCHES "NOVINT_FALCON")
target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${OpenCV_LIBS}
    ${LIBXML2_LIBRARIES}
    Communication
    Video
    Utils
    MasterController
    HapticNovintFalcon
)
endif()
