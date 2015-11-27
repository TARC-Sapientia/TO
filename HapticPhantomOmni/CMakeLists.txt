cmake_minimum_required(VERSION 2.8.0)
project(HapticPhantomOmni)

set(HEADER_FILES
    HapticManagerPhantomOmni.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    HapticManagerPhantomOmni.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

if (UNIX)
  set(HAPTIC_HINTS
      /usr/lib64;
      /usr/lib32;
      /usr/lib;
      /usr/lib/x86_64-linux-gnu)

elseif(WIN32)
		set(HAPTIC_HINTS "")
		set(Haptic_INCLUDE_DIR $ENV{OH_SDK_BASE}/include)
endif(UNIX)

find_library(HD_LIBRARY HD
	PATHS ${HAPTIC_HINTS})
find_library(HL_LIBRARY HL
	PATHS ${HAPTIC_HINTS})

if (UNIX)
  find_library(PHANTOM_DRIVER PHANToMIO
	PATHS ${HAPTIC_HINTS})
endif (UNIX)

message(STATUS Found ${HD_LIBRARY})
message(STATUS Found ${HL_LIBRARY})

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
    ${Haptic_INCLUDE_DIR}
    ${Utils_SOURCE_DIR}
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
    ${HD_LIBRARY}
    ${HL_LIBRARY}
    Utils
)
