# ============================================================================
# Hardware 模块
# ============================================================================
# 聚合相机和串口子模块，提供统一的硬件抽象层

# 子模块
add_subdirectory(hik_cam)
add_subdirectory(serial)

# 硬件节点库
add_library(hardware STATIC
    hardware_node.cpp
)

target_include_directories(hardware PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(hardware PUBLIC
    # 硬件子模块
    hardware_camera
    hardware_serial

    # 第三方库
    fmt::fmt
    tomlplusplus::tomlplusplus
    Eigen3::Eigen
    pybind11::embed
    ${OpenCV_LIBS}
)
