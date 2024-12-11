# 设置源文件
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

# 打印验证
message(STATUS "Source files: ${SOURCES}")
message(STATUS "Header files: ${HEADERS}")

# 定义可执行文件目标
add_executable(Sample ${SOURCES} ${HEADERS})

# 设置头文件路径
target_include_directories(Sample PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加与Library依赖关系
target_link_libraries(Sample PUBLIC Library)


# OpenCV 4.5到4.10版本, 仅CMake3.26以上支持按照版本区间的查找
find_package(OpenCV 4.5...4.10 REQUIRED)

# VTK 8.1到9.4版本
find_package(VTK 8.1...9.4 REQUIRED )
message(STATUS "VTK Libraries: ${VTK_LIBRARIES}")

# PCL 1.8到1.14版本
find_package(PCL 1.8...1.14 REQUIRED)

# 更为推荐的是使用target_include_directories, 这样能灵活控制哪个项目需要哪些库
target_include_directories(Sample PUBLIC
    ${OPENCV_INCLUDE_DIR}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# 为工程添加静态链接库, 库里面是上文include头文件声明函数的具体实现
target_link_libraries(Sample PUBLIC
    ${OpenCV_LIBS} 
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

# 可选：设置输出目录
set_target_properties(Sample PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
