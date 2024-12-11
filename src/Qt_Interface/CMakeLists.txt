set(CMAKE_AUTOMOC ON) # 自动生成moc_*.cpp文件, 连接Qt的信号槽函数
set(CMAKE_AUTORCC ON) # 自动处理qrc文件, 加载图片等资源
set(CMAKE_AUTOUIC ON) # 根据*.ui文件自动生成ui_*.h头文件

# 设置源文件
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")
file(GLOB UI_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.ui")

# 打印验证
message(STATUS "Source files: ${SOURCES}")
message(STATUS "Header files: ${HEADERS}")

# 定义可执行文件目标
add_executable(Qt_Interface ${SOURCES} ${HEADERS} ${UI_FILES})


# 设置头文件路径
target_include_directories(Qt_Interface PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加与Library依赖关系
target_link_libraries(Qt_Interface PUBLIC Library)


# OpenCV 4.5到4.10版本, 仅CMake3.26以上支持按照版本区间的查找
find_package(OpenCV 4.5...4.10 REQUIRED)

# 查找 Qt5 必需模块
find_package(Qt5 COMPONENTS Widgets Core Gui REQUIRED)

# 更为推荐的是使用target_include_directories, 这样能灵活控制哪个项目需要哪些库
target_include_directories(Qt_Interface PUBLIC
    ${OPENCV_INCLUDE_DIR}
)

# 为工程添加静态链接库, 库里面是上文include头文件声明函数的具体实现
target_link_libraries(Qt_Interface PUBLIC
    ${OpenCV_LIBS} 
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
)

# 可选：设置输出目录
set_target_properties(Qt_Interface PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
