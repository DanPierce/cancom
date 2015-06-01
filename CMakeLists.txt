cmake_minimum_required(VERSION 2.8.3)
project(cancom)


################################
## Define locations and files ##
################################

include_directories(${PROJECT_SOURCE_DIR}/include)

set(cancom_HEADERS
	include/cancom.h
)

set(cancom_SRCS
	src/cancom.cpp
  src/cancom_node.cpp
)


############################
## Get necessary packages ##
############################

## Boost
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND cancom_LINK_LIBS
	${Boost_SYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY}
)


####################
## Create library ##
####################

## cancom - primary
add_library(cancom ${cancom_HEADERS} ${cancom_SRCS})
set_target_properties(cancom PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(cancom ${cancom_LINK_LIBS})

#####################
## catkin-specific ##
#####################

option(cancom_BUILD_WITH_CATKIN "Use catkin to build and create a catkin package" on)

if(cancom_BUILD_WITH_CATKIN)

	## find catkin
	find_package(catkin REQUIRED
    COMPONENTS
      can_msgs
      sensor_msgs
      libpcan
      message_generation
      roscpp
      tf
      std_srvs
  )

	include_directories(${catkin_INCLUDE_DIRS})

	## create package
	catkin_package(
		INCLUDE_DIRS ${cancom_INCLUDE_DIRS}
		LIBRARIES cancom
		CATKIN_DEPENDS
      can_msgs
      sensor_msgs
      libpcan
      message_runtime
      roscpp
      tf
      std_srvs
		DEPENDS system_lib
	)

	## create node executable
	add_executable(cancom_node src/cancom_node.cpp)
	target_link_libraries(cancom_node
		cancom
		${cancom_LINK_LIBS}
		${catkin_LIBRARIES}
	)
	# add_dependencies(cancom_node ${PROJECT_NAME}_gencfg) 

endif(cancom_BUILD_WITH_CATKIN)


#############
## INSTALL ##
#############

# Mark executables and/or libraries for installation
if(cancom_BUILD_WITH_CATKIN)
  install(TARGETS cancom cancom_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  # Mark cpp header files for installation
  install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
  )
else(cancom_BUILD_WITH_CATKIN)
  install(TARGETS cancom
    ARCHIVE DESTINATION /usr/local/lib/
    LIBRARY DESTINATION /usr/local/lib/
    RUNTIME DESTINATION /usr/local/bin/${PROJECT_NAME}
  )
  install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION /usr/local/include
  )
  install(FILES Findcancom.cmake DESTINATION ${CMAKE_ROOT}/Modules)
endif(cancom_BUILD_WITH_CATKIN)
