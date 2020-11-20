cmake_minimum_required(VERSION 2.8)
project(stereo_calib)

# 设置编译器编译模式
set( CMAKE_CXX_FLAGS "-std=c++11" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set( CMAKE_BUILD_TYPE "debug" )


# 设置动态链接库输出目录
SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)


# 寻找OpenCV库
#find_package( OpenCV 3 REQUIRED PATHS /usr/local/opencv3.4.5)
find_package( OpenCV 3 REQUIRED )



include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include

)


aux_source_directory(${PROJECT_SOURCE_DIR}/src MYLIB_SRC_DIR)
add_library(MY_LIBS SHARED ${MYLIB_SRC_DIR})
target_link_libraries(MY_LIBS
        ${OpenCV_LIBS}
        )

# stereo calib
add_executable(stereo_calib
        examples/stereo_calib.cpp)

target_link_libraries(
        stereo_calib
        MY_LIBS
)

# generate_sparity
add_executable(gen_sparity
        examples/gen_sparities.cpp)

target_link_libraries(
        gen_sparity
        MY_LIBS
)






