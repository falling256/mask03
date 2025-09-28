cmake_minimum_required(VERSION 3.10)
project(cer3)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 开启多线程支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -pthread")

# 开启 CMAKE_EXPORT_COMPILE_COMMANDS 生成编译命令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 设置 OpenCV 路径，当系统中存在多个 OpenCV 时尤为重要
set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

# 设置 OpenVINO 路径
set(OpenVINO_DIR "/home/shulin/l_openvino_toolkit_ubuntu20_2023.0.0.10926.b4452d56304_x86_64/runtime/cmake")

# 查找库的依赖
find_package(OpenVINO REQUIRED COMPONENTS Runtime)  # 查找 OpenVINO
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# 设置包含路径
include_directories(
    ${OpenVINO_INCLUDE_DIRS}  # OpenVINO 头文件
    ${OpenCV_INCLUDE_DIRS}    # OpenCV 头文件
    ${CERES_INCLUDE_DIRS}     # Ceres 头文件
    /opt/MVS/include
    armor/include
)

# 设置链接库路径
link_directories(
    /opt/MVS/lib/64
    /opt/MVS/lib/32
    /usr/local/lib
)

# 定义所有需要的库（创建一个变量来统一管理）
set(REQUIRED_LIBS
    ${OpenVINO_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

# 创建主程序可执行文件（test_ceres.cpp）
add_executable(test_ceres test_ceres.cpp)

# 链接库
target_link_libraries(test_ceres ${REQUIRED_LIBS})

# 设置宏定义
add_definitions(-DDEBUGMODE)

# 一些常规设置
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
