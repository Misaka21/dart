# ============================================================================
# 项目配置
# ============================================================================
cmake_minimum_required(VERSION 3.16)
project(dart2026 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ============================================================================
# 构建选项
# ============================================================================
option(ENABLE_WEBVIEW "Enable Python webview bindings" ON)
option(USE_CCACHE "Use ccache for faster rebuilds" ON)
option(ENABLE_RERUN "Enable Rerun visualization" OFF)

# ============================================================================
# 编译加速 (ccache)
# ============================================================================
if(USE_CCACHE)
    find_program(CCACHE_PROGRAM ccache)
    if(CCACHE_PROGRAM)
        set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
        message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    endif()
endif()

# ============================================================================
# 编译选项
# ============================================================================
add_compile_options(
    -Wall
    -Wextra
    -Werror=return-type
    -Wno-unused-variable
    -Wno-unused-parameter
)

# 根据构建类型设置优化选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O3 -flto -march=native)
    add_link_options(-flto)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-O3 -flto -march=native -g)
    add_link_options(-flto)
elseif(CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    add_compile_options(-Os -flto -march=native)
    add_link_options(-flto)
else()
    add_compile_options(-O0 -g)
endif()

# ============================================================================
# 路径定义
# ============================================================================
set(CONFIG_DIR "${CMAKE_SOURCE_DIR}/config")
set(LOG_DIR "${CMAKE_SOURCE_DIR}/log")

# 全局编译宏
add_compile_definitions(
    CONFIG_DIR="${CONFIG_DIR}"
    LOG_DIR="${LOG_DIR}"
    $<$<BOOL:${ENABLE_WEBVIEW}>:ENABLE_WEBVIEW>
)

# ============================================================================
# 依赖查找
# ============================================================================
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(Python 3.6 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)

if(ENABLE_RERUN)
    include(FetchContent)
    FetchContent_Declare(rerun_sdk URL
        https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
    FetchContent_MakeAvailable(rerun_sdk)
    add_compile_definitions(ENABLE_RERUN)
endif()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}
)

# ============================================================================
# 子目录
# ============================================================================
add_subdirectory(plugin)
add_subdirectory(hardware)

# ============================================================================
# 主可执行文件
# ============================================================================
add_executable(${PROJECT_NAME}
    main.cpp
    detector/detector.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    # 项目内部库
    plugin
    hardware
    rmcv_bag

    # 第三方库
    fmt::fmt
    tomlplusplus::tomlplusplus
    Eigen3::Eigen
    ${OpenCV_LIBS}
    pybind11::embed

    # 系统库
    Threads::Threads
)

# ============================================================================
# 测试/调试程序
# ============================================================================
add_executable(test_calibration
    test/test_calibration.cpp
)

target_include_directories(test_calibration PRIVATE
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_calibration PRIVATE
    plugin
    hardware
    fmt::fmt
    ${OpenCV_LIBS}
    Threads::Threads
)
