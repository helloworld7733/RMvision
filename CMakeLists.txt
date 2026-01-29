cmake_minimum_required(VERSION 3.10)

project(ArmorDetection CXX)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "D:/Libtools/vcpkg/installed/x64-windows/share/opencv4")
find_package(OpenCV REQUIRED)
add_executable(Detection 
    src/main.cpp 
    src/lightbar_detector.cpp 
    src/armor_matcher.cpp
    src/num_recognizer.cpp
    src/perspective.cpp
)

include_directories(
    ${PROJECT_SOURCE_DIR}/include 
    ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(Detection PRIVATE 
    PROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/"
)
target_link_libraries(Detection ${OpenCV_LIBS})
