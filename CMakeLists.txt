cmake_minimum_required(VERSION 3.0.2)
project(fritzrobot_serial)


find_package(catkin REQUIRED COMPONENTS roscpp serial std_msgs message_generation)

add_message_files(FILES Wheelspeed.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES})
add_dependencies(driver fritzrobot_serial_generate_messages_cpp)

