cmake_minimum_required(VERSION 3.10)  
project(ums_sdk)  
# 设置头文件和库的搜索路径  
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/serial)
include_directories(${CMAKE_SOURCE_DIR}/lib/serial)
set(serial_SRCS
    lib/serial/src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
list(APPEND serial_SRCS lib/serial/src/impl/unix.cc)
list(APPEND serial_SRCS lib/serial/src/impl/list_ports/list_ports_linux.cc)
add_library(serial SHARED ${serial_SRCS})
target_link_libraries(serial rt pthread)



FILE(GLOB SDK_SRC
  "src/*.cpp"
)
# 添加源文件并生成共享库  
add_library(ums_sdk SHARED ${SDK_SRC})  
set_target_properties(ums_sdk PROPERTIES OUTPUT_NAME "ums_sdk")  
target_link_libraries(ums_sdk serial)

# 设置编译器选项  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")