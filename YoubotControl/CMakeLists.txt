cmake_minimum_required(VERSION 2.8.0)

project (YouBot)

set(HEADER_FILES
   YouBotControl.hpp
   Slave_Controller_YouBot.h
   PO_PC_Slave_YoubotArm.h
   Slave_Controller.h
)

source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES 
    YouBotControl.cpp
    Slave_Controller_YouBot.cpp
    PO_PC_Slave_YoubotArm.cpp
    Slave_Controller.cpp
)

source_group("Source Files" FILES ${SOURCE_FILES})

include_directories(
    ${YOUBOT_INCLUDE}
    ${ETHERCAT_INCLUDE}
    ${Communication_SOURCE_DIR}
    ${LIBXML2_INCLUDE_DIR}
    ${Utils_SOURCE_DIR}
)

add_library(
    ${PROJECT_NAME}
    ${SOURCE_FILES}
    ${HEADER_FILES}
)

target_link_libraries(
    ${PROJECT_NAME}
    ${YOUBOT_DRIVER}
    ${LIBSOEM}
    Communication
)
