cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [~/FindOpenCV.cmake])
project(opencv_coordinate_package)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy rosbag std_msgs genmsg message_generation)
find_package(OpenCV)




## Declare ROS messages and services
add_message_files(FILES fObject.msg fObjectArray.msg fStat.msg)
##add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} . src/)
link_directories(.)

add_executable(talker coord_publisher.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)


add_executable(listener coord_subscriber.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

add_executable(vtalker velo_subpub.cpp)
target_link_libraries(vtalker ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(vtalker beginner_tutorials_generate_messages_cpp)

add_executable(vlistener velo_testsub.cpp)
target_link_libraries(vlistener ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(vlistener beginner_tutorials_generate_messages_cpp)

add_executable(filterCalibration filterCalibrationROS.cpp)
target_link_libraries(filterCalibration ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(filterCalibration beginner_tutorials_generate_messages_cpp)

add_executable(clickMove clickmove_subpub.cpp)
target_link_libraries(clickMove ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(clickMove beginner_tutorials_generate_messages_cpp)

#add_executable(vsrv velocity_server.cpp)
#target_link_libraries(vsrv ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(vsrv beginner_tutorials_generate_messages_cpp)

