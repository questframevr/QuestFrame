cmake_minimum_required(VERSION 3.22.1)
project(questvr CXX)

find_package(OpenXR REQUIRED CONFIG)

add_library(questvr SHARED questvr.cpp)

target_link_libraries(questvr OpenXR::openxr_loader EGL GLESv3 android nativewindow log)

target_compile_definitions(questvr PRIVATE
    XR_USE_PLATFORM_ANDROID=1
    XR_USE_GRAPHICS_API_OPENGL_ES=1
)
target_compile_features(questvr PRIVATE cxx_std_17)
target_compile_options(questvr PRIVATE -Wall -Wextra -Werror=return-type -Wno-missing-field-initializers)
