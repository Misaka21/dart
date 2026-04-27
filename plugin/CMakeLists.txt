cmake_minimum_required(VERSION 3.16)
set(lib_name "plugin")

set(plugin_param_src
    param/runtime_parameter.cpp
    param/parameter.cpp
)

set(plugin_visualizer_src
    visualizer/visualizer_node.cpp
)

set(plugin_telemetry_src
    telemetry/telemetry.cpp
)

add_library(${lib_name} STATIC
    ${plugin_param_src}
    ${plugin_visualizer_src}
    ${plugin_telemetry_src}
)

target_include_directories(${lib_name} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${lib_name} PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    fmt::fmt
)

if(ENABLE_WEBVIEW)
    target_link_libraries(${lib_name} PUBLIC pybind11::embed)
endif()

add_subdirectory(rmcv_bag)

if(ENABLE_RERUN)
    add_subdirectory(rerun)
    target_link_libraries(${lib_name} PUBLIC rerun_plugin)
endif()
