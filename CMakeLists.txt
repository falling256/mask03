cmake_minimum_required(VERSION 3.10)
project(cer3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 开启多线程支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -pthread")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)

set(OpenVINO_DIR "/home/shulin/l_openvino_toolkit_ubuntu20_2023.0.0.10926.b4452d56304_x86_64/runtime/cmake")

# 查找库的依赖
find_package(OpenVINO REQUIRED COMPONENTS Runtime)  # 查找 OpenVINO
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

# 包含路径
include_directories(
    ${OpenVINO_INCLUDE_DIRS}  # OpenVINO 头文件
    ${OpenCV_INCLUDE_DIRS}    # OpenCV 头文件
    ${CERES_INCLUDE_DIRS}     # Ceres 头文件
    /opt/MVS/include
    armor/include
)

# 链接库路径
link_directories(
    /opt/MVS/lib/64
    /opt/MVS/lib/32
    /usr/local/lib
)

set(REQUIRED_LIBS
    ${OpenVINO_LIBRARIES}
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

add_executable(test_ceres test_ceres.cpp)
target_link_libraries(test_ceres ${REQUIRED_LIBS})

add_definitions(-DDEBUGMODE)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
