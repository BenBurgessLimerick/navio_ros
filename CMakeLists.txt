cmake_minimum_required(VERSION 2.8.3)
project(navio_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navio_ros

  CATKIN_DEPENDS 
	roscpp
)


include_directories(
	include/navio_ros
	include/navio_ros/Common
	include/navio_ros/Navio+
	include/navio_ros/Navio2
 	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(navio_ros
	src/navio_ros/Common/gpio.cpp
	src/navio_ros/Common/I2Cdev.cpp
	src/navio_ros/Common/MPU9250.cpp
	src/navio_ros/Common/MS5611.cpp
	src/navio_ros/Common/Ublox.cpp
	src/navio_ros/Common/Util.cpp
	src/navio_ros/Common/AHRS.cpp
	
	src/navio_ros/Navio2/ADC_Navio2.cpp
	src/navio_ros/Navio2/LSM9DS1.cpp
	src/navio_ros/Navio2/RCInput_Navio2.cpp
	src/navio_ros/Navio2/RGBled.cpp
	src/navio_ros/Navio2/Led_Navio2.cpp
	src/navio_ros/Navio2/PWM.cpp
	src/navio_ros/Navio2/RCOutput_Navio2.cpp
)

add_executable(led_node src/navio_ros/led_node.cpp 
	src/navio_ros/Navio2/Led_Navio2.cpp 
	src/navio_ros/Navio2/RGBled.cpp
	src/navio_ros/Common/gpio.cpp)
add_dependencies(led_node navio_ros)

add_executable(gps_node src/navio_ros/gps_node.cpp
		src/navio_ros/Common/Util.cpp
		src/navio_ros/Common/Ublox.cpp)
add_dependencies(gps_node navio_ros)
target_link_libraries(gps_node ${catkin_LIBRARIES})


add_executable(imu_node src/navio_ros/imu_node.cpp
		src/navio_ros/Common/MPU9250.cpp
		src/navio_ros/Common/AHRS.cpp
		src/navio_ros/Common/Util.cpp
		src/navio_ros/Navio2/LSM9DS1.cpp)
add_dependencies(imu_node navio_ros)
target_link_libraries(imu_node ${catkin_LIBRARIES})
