cmake_minimum_required(VERSION 3.0.2)
project(fritzrobot_serial)


find_package(catkin REQUIRED COMPONENTS roscpp serial std_msgs message_generation)

add_message_files(FILES Wheelspeed.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} include/fritzrobot_serial/ include/fritzrobot_serial/Eigen)

add_executable(driver src/driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES})
add_dependencies(driver fritzrobot_serial_generate_messages_cpp)

add_executable(pose_estimation src/pose_estimation.cpp
        src/ekf.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES})

add_executable(downstream src/downstream.cpp)
target_link_libraries(downstream ${catkin_LIBRARIES})
