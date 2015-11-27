cmake_minimum_required(VERSION 2.8.0)
project(Teleoperation)

set(CMAKE_SKIP_RPATH ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# The CMAKE_PREFIX_PATH must contain all include paths 
# The CMAKE_LIBRARY_PATH must contain all library paths

if (WIN32)
    set(CMAKE_VERBOSE_MAKEFILE ON)

    # Add path to opencv static libs
    set(CMAKE_PREFIX_PATH "$ENV{LIBDIR}/Debug/x86/vc10/staticlib;${CMAKE_PREFIX_PATH}")

    # Add path to project includes: boost, opencv, libxml
    set(CMAKE_PREFIX_PATH "$ENV{LIBDIR}/debug/include;${CMAKE_PREFIX_PATH}")

	if (${HAPTIC} MATCHES "PHANTOM_OMNI")
		# Add path to Open Haptics SDK include and libraries
		
		# The default {OH_SDK_BASE} install directory is: C:\OpenHaptics\Developer\3.4.0
		
		set(CMAKE_PREFIX_PATH "$ENV{OH_SDK_BASE}/;${CMAKE_PREFIX_PATH}")
		set(CMAKE_LIBRARY_PATH "$ENV{OH_SDK_BASE}/lib/Win32/Release;${CMAKE_LIBRARY_PATH}")		
    endif()
	
	if (${HAPTIC} MATCHES "NOVINT_FALCON")
	    set(CMAKE_PREFIX_PATH "C:/Program Files (x86)/Novint/HDAL_SDK_2.1.3/include;${CMAKE_PREFIX_PATH}")
		set(CMAKE_LIBRARY_PATH "C:/Program Files (x86)/Novint/HDAL_SDK_2.1.3/lib;${CMAKE_LIBRARY_PATH}")
		
	    set(CMAKE_PREFIX_PATH "C:/Program Files/Novint/HDAL_SDK_2.1.3/include;${CMAKE_PREFIX_PATH}")
		set(CMAKE_LIBRARY_PATH "C:/Program Files/Novint/HDAL_SDK_2.1.3/lib;${CMAKE_LIBRARY_PATH}")			
    endif()
	
	# All additional library dependencies
    set(CMAKE_LIBRARY_PATH "$ENV{LIBDIR}/Debug/lib/;${CMAKE_LIBRARY_PATH}")

    add_definitions(-DBOOST_ALL_NO_LIB)
    add_definitions(-DCMAKE_VERBOSE_MAKEFILE=1)

    set(Boost_USE_STATIC_LIBS ON)
    set(Boost_USE_MULTITHREADED ON)
    set(Boost_USE_STATIC_RUNTIME OFF)

    add_definitions(-D_WIN32_WINNT=0x0600)

	#message(status "**!!! CMAKE_LIBRARY_PATH  ${CMAKE_LIBRARY_PATH}")
	#message(status "**!!! CMAKE_PREFIX_PATH  ${CMAKE_PREFIX_PATH}")

endif()

# NOTE: This is the order of dependency

add_subdirectory(Utils)
add_subdirectory(Communication)
add_subdirectory(Video)

if (${CONFIGURATION} MATCHES "MASTER")
    message(STATUS "Building MASTER side ...")

    # Select haptic device
    if (${HAPTIC} MATCHES "OFF")
        add_definitions(-DHAPTIC_OFF)
        message(STATUS "No haptic device selected. Data will be generated ...")
    endif()

    if (${HAPTIC} MATCHES "PHANTOM_OMNI")
        add_definitions(-DHAPTIC_PHANTOM_OMNI)
        add_subdirectory(HapticPhantomOmni)

        message(STATUS "Haptic device: Phantom Omni is selected ...")
    endif()

    if (${HAPTIC} MATCHES "NOVINT_FALCON")
        add_definitions(-DHAPTIC_NOVINT_FALCON)
        add_subdirectory(HapticNovintFalcon)

        message(STATUS "Haptic device: Novint Falcon is selected ...")
    endif()

    add_subdirectory(MasterController)

    add_subdirectory(Master)
endif()

if (${CONFIGURATION} MATCHES "SLAVE")
    message(STATUS "Building SLAVE side ...")

    if (${YOUBOT} MATCHES "OFF")
        add_definitions(-DYOUBOT_OFF)

    else()

        add_definitions(-DYOUBOT_ON)

        # Path of the youbot_driver
        set(YOUBOT_PATH "$ENV{HOME}/youbot_driver")

        set(YOUBOT_INCLUDE "${YOUBOT_PATH}")
        set(ETHERCAT_INCLUDE "${YOUBOT_PATH}/soem/src/")
        set(YOUBOT_DRIVER "${YOUBOT_PATH}/lib/libYouBotDriver.a")
        set(LIBSOEM "${YOUBOT_PATH}/lib/libsoem.a")

        add_definitions(-DYouBot_CONFIG_DIR="${YOUBOT_PATH}/config")

        add_subdirectory(YoubotControl)

        message(STATUS "The Youbot driver is included.")

    endif()

    add_subdirectory(Slave)

endif()
