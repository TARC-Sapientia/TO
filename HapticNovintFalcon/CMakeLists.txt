cmake_minimum_required(VERSION 2.8.0)

project(HapticNovintFalcon)

set(HEADER_FILES
	HapticManagerNovintFalcon.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
	HapticManagerNovintFalcon.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED)

if (UNIX)
  set(HAPTIC_HINTS
      /usr/lib64;
      /usr/lib32;
      /usr/lib;
      /usr/lib/x86_64-linux-gnu)
else ()
	set(Haptic_INCLUDE_DIR "C:/Program Files (x86)/Novint/HDAL_SDK_2.1.3/include;${Haptic_INCLUDE_DIR}")
	set(Haptic_LIBRARY_DIR "C:/Program Files (x86)/Novint/HDAL_SDK_2.1.3/lib;${Haptic_LIBRARY_DIR}")

	set(Haptic_INCLUDE_DIR "C:/Program Files/Novint/HDAL_SDK_2.1.3/include;${Haptic_INCLUDE_DIR}")
	set(Haptic_LIBRARY_DIR "C:/Program Files/Novint/HDAL_SDK_2.1.3/lib;${Haptic_LIBRARY_DIR}")
	
endif(UNIX)

find_library(HDL_LIBRARY HDL
	PATHS ${Haptic_LIBRARY_DIR})

message(STATUS Found ${HDL_LIBRARY})

include_directories( 
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}
	${Haptic_INCLUDE_DIR}
	${Utils_SOURCE_DIR}
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
	${HDL_LIBRARY}
	Utils
)
