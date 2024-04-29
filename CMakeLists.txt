cmake_minimum_required(VERSION 3.10)
project(ums_sdk)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 设置头文件和库的搜索路径
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/include/serial)
include_directories(${CMAKE_SOURCE_DIR}/include/log4cpp)
#include_directories(${CMAKE_SOURCE_DIR}/lib/serial)


    if(APPLE)
            find_library(IOKIT_LIBRARY IOKit)
            find_library(FOUNDATION_LIBRARY Foundation)
    endif()
    if(UNIX AND NOT APPLE)
            # If Linux, add rt and pthread
            set(rt_LIBRARIES rt)
            set(pthread_LIBRARIES pthread)
    else()
            # Otherwise normal call
    endif()
    ## Sources
    set(serial_SRCS
            lib/serial/src/serial.cc
            include/serial/serial.h
            include/serial/v8stdint.h
)
    if(APPLE)
        # If OSX
        list(APPEND serial_SRCS  lib/serial/src/impl/unix.cc)
        list(APPEND serial_SRCS  lib/serial/src/impl/list_ports/list_ports_osx.cc)
    elseif(UNIX)
        # If unix
        list(APPEND serial_SRCS  lib/serial/src/impl/unix.cc)
        list(APPEND serial_SRCS lib/serial/src/impl/list_ports/list_ports_linux.cc)
    else()
        # If windows
        list(APPEND serial_SRCS  lib/serial/src/impl/win.cc)
        list(APPEND serial_SRCS  lib/serial/src/impl/list_ports/list_ports_win.cc)
    endif()

    ## Add serial library
    add_library(serial SHARED ${serial_SRCS})
    if(APPLE)
        target_link_libraries(serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
    elseif(UNIX)
        target_link_libraries(serial rt pthread)
    else()
        target_link_libraries(serial setupapi)
    endif()



    IF (WIN32)
      SET (CMAKE_DEBUG_POSTFIX "d")
      LIST(APPEND LOG4CPP_CFLAGS -D_CRT_SECURE_NO_WARNINGS -DLOG4CPP_HAS_DLL -DLOG4CPP_BUILD_DLL )
    ELSE (WIN32)
      IF (APPLE)
    #	LIST(APPEND LOG4CPP_CFLAGS -DLOG4CPP_HAVE_SSTREAM)
      ELSE (APPLE)
	    LIST(APPEND LOG4CPP_CFLAGS -pthread)
      ENDIF (APPLE)
    ENDIF (WIN32)


set(LOG4_SRCS
        lib/log4cpp/src/Appender.cpp
        lib/log4cpp/src/AppenderSkeleton.cpp
        lib/log4cpp/src/AppendersFactory.cpp
        lib/log4cpp/src/BufferingAppender.cpp
        lib/log4cpp/src/FactoryParams.cpp
        lib/log4cpp/src/LayoutsFactory.cpp
        lib/log4cpp/src/LevelEvaluator.cpp
        lib/log4cpp/src/Localtime.cpp
        lib/log4cpp/src/PassThroughLayout.cpp
        lib/log4cpp/src/TriggeringEventEvaluatorFactory.cpp
        lib/log4cpp/src/LayoutAppender.cpp
        lib/log4cpp/src/FileAppender.cpp
        lib/log4cpp/src/DailyRollingFileAppender.cpp
        lib/log4cpp/src/RollingFileAppender.cpp
        lib/log4cpp/src/GenerationalFileAppender.cpp
        lib/log4cpp/src/FixedContextCategory.cpp
        lib/log4cpp/src/IdsaAppender.cpp
        lib/log4cpp/src/OstreamAppender.cpp
        lib/log4cpp/src/StringQueueAppender.cpp
        lib/log4cpp/src/SyslogAppender.cpp
        lib/log4cpp/src/RemoteSyslogAppender.cpp
        lib/log4cpp/src/SimpleLayout.cpp
        lib/log4cpp/src/BasicLayout.cpp
        lib/log4cpp/src/PatternLayout.cpp
        lib/log4cpp/src/Category.cpp
        lib/log4cpp/src/CategoryStream.cpp
        lib/log4cpp/src/HierarchyMaintainer.cpp
        lib/log4cpp/src/Configurator.cpp
        lib/log4cpp/src/BasicConfigurator.cpp
        lib/log4cpp/src/SimpleConfigurator.cpp
        lib/log4cpp/src/PropertyConfigurator.cpp
        lib/log4cpp/src/PropertyConfiguratorImpl.cpp
        lib/log4cpp/src/LoggingEvent.cpp
        lib/log4cpp/src/Priority.cpp
        lib/log4cpp/src/NDC.cpp
        lib/log4cpp/src/Filter.cpp
        lib/log4cpp/src/TimeStamp.cpp
        lib/log4cpp/src/StringUtil.cpp
        lib/log4cpp/src/Properties.cpp
        lib/log4cpp/src/Win32DebugAppender.cpp
        lib/log4cpp/src/NTEventLogAppender.cpp
        lib/log4cpp/src/DllMain.cpp
        lib/log4cpp/src/DummyThreads.cpp
        lib/log4cpp/src/MSThreads.cpp
        lib/log4cpp/src/OmniThreads.cpp
        lib/log4cpp/src/PThreads.cpp
        lib/log4cpp/src/PortabilityImpl.cpp
        lib/log4cpp/src/AbortAppender.cpp
)

add_library(log4 SHARED ${LOG4_SRCS})
IF (WIN32)
  TARGET_LINK_LIBRARIES (log4 kernel32 user32 ws2_32 advapi32 )
#  SET_TARGET_PROPERTIES(${LOG4CPP_LIBRARY_NAME} PROPERTIES LINK_FLAGS /NODEFAULTLIB:msvcrt)
ENDIF (WIN32)
if(UNIX)
target_link_libraries(log4 pthread)
ENDIF (UNIX)


if(UNIX)
FILE(GLOB SDK_SRC
  "src/battery.cpp"
  "src/crc.cpp"
  "src/queue.cpp"
  "src/ums_serial_methods.cpp"
)
endif(UNIX)
if(WIN32)
FILE(GLOB SDK_SRC
  "src/battery.cpp"
  "src/crc.cpp"
  "src/queue.cpp"
  "src/ums_serial_methods_win.cpp"
)
endif(WIN32)

# 添加源文件并生成共享库
add_library(ums_sdk SHARED ${SDK_SRC})
set_target_properties(ums_sdk PROPERTIES OUTPUT_NAME "ums_sdk")
target_link_libraries(ums_sdk serial log4)


add_executable(example
        example/main.cpp
)
target_link_libraries(example ums_sdk serial log4)



# 设置编译器选项  
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")