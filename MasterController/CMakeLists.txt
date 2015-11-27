project(MasterController)

set(HEADER_FILES
    Master_Controller.h
    Master_Controller_Youbot.h
    PO_PC_Master_YoubotArm.h
)
source_group("Header Files" FILES ${HEADER_FILES})

set(SOURCE_FILES
    Master_Controller.cpp
    Master_Controller_Youbot.cpp
    PO_PC_Master_YoubotArm.cpp
)
source_group("Source Files" FILES ${SOURCE_FILES})

find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR}
    ${Utils_SOURCE_DIR}
    ${Communication_SOURCE_DIR}
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
    Utils
)