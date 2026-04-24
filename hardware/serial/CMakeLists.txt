# ============================================================================
# Serial 模块
# ============================================================================
# 串口通信模块，支持 UART 和 USB Bulk 传输

# 源文件
file(GLOB SERIAL_SOURCES "*.cpp")
file(GLOB PROTOCOL_SOURCES "protocol/*.cpp")

add_library(hardware_serial STATIC
    ${SERIAL_SOURCES}
    ${PROTOCOL_SOURCES}
)

target_include_directories(hardware_serial PUBLIC
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ============================================================================
# libusb-1.0 依赖 (可选，用于 USB Bulk 传输)
# ============================================================================
find_path(LIBUSB_INCLUDE_DIR
    NAMES libusb.h
    PATHS
        /usr/include/libusb-1.0
        /usr/local/include/libusb-1.0
        /opt/homebrew/include/libusb-1.0
    PATH_SUFFIXES libusb-1.0
)

find_library(LIBUSB_LIBRARY
    NAMES usb-1.0 libusb-1.0
    PATHS
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        /opt/homebrew/lib
)

if(LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARY)
    message(STATUS "Found libusb-1.0: ${LIBUSB_LIBRARY}")
    target_include_directories(hardware_serial PRIVATE ${LIBUSB_INCLUDE_DIR})
    target_link_libraries(hardware_serial PRIVATE ${LIBUSB_LIBRARY})
    target_compile_definitions(hardware_serial PRIVATE HAVE_LIBUSB_1_0)
else()
    message(WARNING "libusb-1.0 not found, USB bulk transfer disabled")
endif()

# ============================================================================
# 系统库
# ============================================================================
target_link_libraries(hardware_serial PRIVATE
    pthread
    fmt::fmt
    Eigen3::Eigen
    pybind11::embed
)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(hardware_serial PRIVATE rt)
endif()
