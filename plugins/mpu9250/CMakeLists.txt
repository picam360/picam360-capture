cmake_minimum_required(VERSION 3.1.3)

message("mpu9250 generating Makefile")
project(mpu9250)

add_library(mpu9250 MODULE
	mpu9250.c
)
set_target_properties(mpu9250 PROPERTIES
    C_STANDARD 11
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS ON # gnu11
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO # c++11
    PREFIX ""
)

add_custom_target(I2Cdev_make
	COMMAND make -C I2Cdev
)
add_library(I2Cdev STATIC IMPORTED GLOBAL)
add_dependencies(I2Cdev I2Cdev_make)

set_target_properties(I2Cdev
    PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/libI2Cdev.a)

add_custom_target(MotionSensor_make
	COMMAND make -C MotionSensor
)
add_library(MotionSensor STATIC IMPORTED GLOBAL)
add_dependencies(MotionSensor MotionSensor_make)

set_target_properties(MotionSensor
    PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/MotionSensor/libMotionSensor.a)

	
include_directories(
	../../include
	MotionSensor
)
link_directories(
)

target_link_libraries(mpu9250
	MotionSensor
	I2Cdev
	pthread
	dl
)

#for pthread_setname_np
add_definitions(-D_GNU_SOURCE)

#post build
add_custom_command(TARGET mpu9250 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:mpu9250> ${CMAKE_CURRENT_SOURCE_DIR}/../
)