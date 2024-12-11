cmake_minimum_required(VERSION 3.11)

# 
set(CMAKE_PROJECT_NAME "OpticalMagneticNavigation")

# 
project(${CMAKE_PROJECT_NAME})

# 添加子项目
add_subdirectory(Library)
add_subdirectory(Sample)
add_subdirectory(Qt_Interface)
