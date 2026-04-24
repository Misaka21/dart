# ============================================================================
# HIK Camera 模块
# ============================================================================
# 海康威视工业相机驱动

set(HIK_MVS_PATH "/opt/MVS")

# 源文件
file(GLOB_RECURSE HIK_CAM_SOURCES "*.cpp")

add_library(hardware_camera STATIC ${HIK_CAM_SOURCES})

# 包含目录
target_include_directories(hardware_camera PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${HIK_MVS_PATH}/include
)

# 根据架构选择 MVS 库路径
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(HIK_LIB_PATH "${HIK_MVS_PATH}/lib/64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(HIK_LIB_PATH "${HIK_MVS_PATH}/lib/aarch64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

target_link_directories(hardware_camera PUBLIC ${HIK_LIB_PATH})

target_link_libraries(hardware_camera PUBLIC
    MvCameraControl
    ${OpenCV_LIBS}
    fmt::fmt
    Eigen3::Eigen
)
