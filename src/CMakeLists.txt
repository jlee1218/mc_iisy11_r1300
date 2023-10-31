# Here we find a typical ROS description package, this set the
# JVRC_DESCRIPTION_PATH CMake variable
find_description_package(mc_iisy11_r1300_description)

if(EXISTS "${MC_IISY11_R1300_DESCRIPTION_PATH}/share/mc_iisy11_r1300_description")
  set(MC_UR5E_DESCRIPTION_PATH "${MC_IISY11_R1300_DESCRIPTION_PATH}/share/mc_iisy11_r1300_description")
endif()

# Generate the robot module C++ code using this value
configure_file(MyRobotModule.in.cpp
               "${CMAKE_CURRENT_BINARY_DIR}/MyRobotModule.cpp")

set(module_SRC "${CMAKE_CURRENT_BINARY_DIR}/MyRobotModule.cpp")

set(module_HDR MyRobotModule.h)

add_robot(MyRobotModule "${module_SRC}" "${module_HDR}")
target_include_directories(MyRobotModule PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
