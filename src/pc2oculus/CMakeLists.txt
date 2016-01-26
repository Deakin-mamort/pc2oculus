#Required CMake version
cmake_minimum_required(VERSION 2.8.3)
#Name of package
project(pc2oculus)

#Find the caktin build system, and any other packages on which we depend
find_package(Qt4 4.4.3 REQUIRED QtGui QtXml)
find_package( PCL REQUIRED )
find_package(catkin REQUIRED COMPONENTS 
			 rviz
			)
#Declare our catkin package
catkin_package(
		INCLUDE_DIRS include)

#Specify locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )


#Specify locations of link files
link_directories(${PCL_LIBRARY_DIRS})

#Declare definitions
add_definitions(${tinyxml_cflags})
add_definitions(${PCL_DEFINITIONS})

#Declare the executable (multi line for multi executables)
add_executable(pc2oculus src/pc2oculus.cpp)

#Specify libraries against which to link (line for each executable)
target_link_libraries(pc2oculus ${catkin_LIBRARIES})
target_link_libraries(pc2oculus Qt4::QtGui Qt4::QtXml)
target_link_libraries(pc2oculus ${PCL_LIBRARIES} ${catkin_LIBRARIES})



