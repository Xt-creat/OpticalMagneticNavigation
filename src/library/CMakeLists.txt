# 最低版本要求
cmake_minimum_required(VERSION 3.11)

# 动态库项目名称，这告诉 CMake 创建一个动态库，而不是静态库（STATIC）或模块库（MODULE）。
add_library(Library SHARED)

# 编译器设置
set_target_properties(Library PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON  # 动态库需要位置无关代码
    DEFINE_SYMBOL CAPICOMMON_EXPORTS # 设置自定义导出宏
)

set(VTK_DIR " D:/Env/VTK9.4/install/lib/cmake/vtk-9.4")

# 设置源文件和头文件路径
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB HEADERS 
    "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/include/*.h"
)

# 打印验证
message(STATUS "Source files: ${SOURCES}")
message(STATUS "Header files: ${HEADERS}")

# 添加源文件和头文件到库目标
target_sources(Library PRIVATE ${SOURCES} ${HEADERS})


# OpenCV 4.5到4.10版本, 仅CMake3.26以上支持按照版本区间的查找
find_package(OpenCV 4.5...4.10 REQUIRED)

# 查找 Eigen3 库
find_package(Eigen3 REQUIRED)

# VTK 8.1到9.5版本
find_package(VTK 8.1...9.5 REQUIRED )
message(STATUS "VTK Libraries: ${VTK_LIBRARIES}")

# 设置头文件目录
target_include_directories(Library PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${EIGEN3_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
)

# 可选：设置输出目录
set_target_properties(Library PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)


# 平台检查 (macOS 和其他平台)
if(APPLE)
    set(OPENSSL_INC "/usr/local/opt/openssl@1.1/include")
    set(OPENSSL_LIB "/usr/local/opt/openssl@1.1/lib")
    target_include_directories(Library PUBLIC ${OPENSSL_INC})
    target_link_directories(Library PUBLIC ${OPENSSL_LIB})
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-install_name,@rpath/${CMAKE_SHARED_LIBRARY_NAME}")
else()
    set(OPENSSL_INC "/usr/include")
    target_include_directories(Library PUBLIC ${OPENSSL_INC})
endif()

# 链接依赖库
#target_link_libraries(Library PUBLIC
#    pthread  # 线程支持
#    crypto   # OpenSSL 加密库
#)

target_link_libraries(Library PUBLIC
    ${OpenCV_LIBS} 
    ${VTK_LIBRARIES}
)

# 自定义清理目标（可选）
add_custom_target(clean-all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}
    COMMENT "Cleaning all build files..."
)
