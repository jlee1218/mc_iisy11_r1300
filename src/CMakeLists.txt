# Here we find a typical ROS description package, this set the
# IISY11_R1300_DESCRIPTION_PATH CMake variable
find_description_package(mc_iisy11_r1300_description)

if(EXISTS "${MC_IISY11_R1300_DESCRIPTION_PATH}/share/mc_iisy11_r1300_description")
  set(MC_IISY11_R1300_DESCRIPTION_PATH "${MC_IISY11_R1300_DESCRIPTION_PATH}/share/mc_iisy11_r1300_description")
endif()

# Generate the robot module C++ code using this value
configure_file(Iisy11r1300Module.in.cpp
               "${CMAKE_CURRENT_BINARY_DIR}/Iisy11r1300Module.cpp")

set(iisy11_r1300_SRC "${CMAKE_CURRENT_BINARY_DIR}/Iisy11r1300Module.cpp")

set(iisy11_r1300_HDR Iisy11r1300Module.h)

add_robot(iisy11_r1300 "${iisy11_r1300_SRC}" "${iisy11_r1300_HDR}")
target_include_directories(Iisy11r1300Module PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
