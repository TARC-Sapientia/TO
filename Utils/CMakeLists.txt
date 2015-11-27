cmake_minimum_required(VERSION 2.8.0)

project(Utils)

set(HEADER_FILES
    Core.h
    IXmlParser.h
    XmlParser.h
    IHapticXmlParser.h
    HapticXmlParser.h
    Utilities.h
    Log.h	
    IKukaXmlParser.h
    KukaXmlParser.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    Core.cpp
    XmlParser.cpp
    HapticXmlParser.cpp
    Log.cpp	
    KukaXmlParser.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)
find_package(LibXml2 REQUIRED)

include_directories( 
    ${Boost_INCLUDE_DIRS}
    ${LIBXML2_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}
)

add_definitions(${LIBXML2_DEFINITIONS})

add_library(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${Boost_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    ${LIBXML2_LIBRARIES}
)
