cmake_minimum_required(VERSION 3.5)
project(add_cloud)

project(${PROJECT_NAME} C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CXX_SOURCE_FILE_EXTS cpp;cxx)
set(CXX_INCLUDE_FILE_EXTS hpp;h)
set(CMAKE_C_COMPILER gcc)
set(CMAKE_CXX_COMPILER g++)

find_package(PCL REQUIRED)

include_directories(
   include
   ${PCL_INCLUDE_DIRS}
)

add_executable(add_cloud
   src/add_cloud.cpp
)

target_link_libraries(add_cloud
    ${PCL_LIBRARIES}
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} -lpthread -lm)
