cmake_minimum_required(VERSION 3.10)
project(ylo2_legged_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        legged_hw
        )

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        legged_hw
)

###########
## Build ##
###########

# Motors driver
#include_directories(
#        moteus_driver/include
#        include
#        ${catkin_INCLUDE_DIRS}
#)
# Motors driver
#add_subdirectory(moteus_driver)

#target_link_libraries(moteus_driver ${catkin_LIBRARIES})

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

link_directories(lib)

## Declare cpp executables
add_executable(${PROJECT_NAME}
        src/ylo2_legged_hw.cpp
        src/ylo2hw.cpp
        )

## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )

endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

# Mark other files for installation
install(DIRECTORY config launch lib
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
