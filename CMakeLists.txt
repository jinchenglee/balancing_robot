project(balancing_robot)

cmake_minimum_required (VERSION 3.5)

find_package (Threads REQUIRED) 

set(sources
    src/bcm2835/bcm2835.c 
    src/motor/motor.cpp 
    src/I2Cdev/I2Cdev.cpp 
    src/imu/MPU6050.cpp 
    src/imu/imu.cpp 
    src/main.cpp 
    )

add_executable(balancing_robot ${sources})

target_compile_options(balancing_robot PUBLIC -std=c++0x -Wall)

target_link_libraries(balancing_robot cppgpio Threads::Threads)

target_include_directories(balancing_robot PUBLIC src)
