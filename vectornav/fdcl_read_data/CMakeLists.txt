cmake_minimum_required(VERSION 2.8.4)
project(fdcl_read_data)

set(CMAKE_SUPPRESS_REGENERATION TRUE)

add_subdirectory(../cpp libvncxx)

include_directories(../cpp/include)

if(CMAKE_COMPILER_ID MATCHES "GNU|Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()


################################################################################
#### Compile Executable
################################################################################

set(SOURCE_FILES
    main.cpp)

add_executable(fdcl_read_data ${SOURCE_FILES})
target_link_libraries(fdcl_read_data PRIVATE libvncxx vn100)


################################################################################
#### Compile VectorNav libraries
################################################################################

set(vn100_src
    fdcl_vn100.cpp
)
add_library(vn100 STATIC ${vn100_src})

if (UNIX OR APPLE)
	target_link_libraries(vn100 LINK_PUBLIC pthread)
else()
	target_link_libraries(vn100 LINK_PUBLIC Advapi32 SetupAPI)
endif()

target_link_libraries(vn100 PRIVATE libvncxx)
