#设置最低版本的cmake要求
cmake_minimum_required(VERSION 3.13)
#设置项目信息
#set(PROJECT_NAME opencv_test)
#project(${PROJECT_NAME})
project(opencv_test)


#寻找opencv库
find_package(OpenCV)
#库的包含目录
include_directories(${OpenCV_INCLUDE_DIRS})


#打印调试信息
MESSAGE(STATUS "Project:opencv_test")
MESSAGE(STATUS "Project Dirrectory: ${PROJECT_SOURCE_DIR}")

#设置调试模式，debug会输出调试信息
SET(CMAKE_BUILD_TYPE DEBUG)

#设置编译器版本
set(CMAKE_CXX_STANDARD 14)

#设置源文件（主文件名）
set(SOURCE_FILES main.cpp)

#设置要生成可执行程序的文件名
add_executable(opencv_test ${SOURCE_FILES})

#添加自己写的其他函数文件，注意这个名字不能和工程名冲突
ADD_LIBRARY(myfile STATIC rotate.cpp splice_demo.cpp PerspectiveTrans.cpp
        include_files.h ChangeDefine.h)

#设置链接库
target_link_libraries(opencv_test ${OpenCV_LIBS} myfile)
#target_link_libraries(opencv_test ${OpenCV_LIBS})
