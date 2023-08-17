cmake_minimum_required(VERSION 3.13)

add_library(xinput_host INTERFACE)
target_sources(xinput_host INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/xinput_host.c
)
target_include_directories(xinput_host INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(xinput_host INTERFACE pico_stdlib tinyusb_host)

add_library(xinput_host_app_driver INTERFACE)
target_sources(xinput_host_app_driver INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/xinput_host_app_driver.c
)
target_include_directories(xinput_host_app_driver INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(xinput_host_app_driver INTERFACE xinput_host)