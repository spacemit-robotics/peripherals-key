cmake_minimum_required(VERSION 3.10)

project(key_lib)

# Standalone default (and compatibility with exported targets).
if(NOT DEFINED SROBOTIS_EXPORT_SET)
    set(SROBOTIS_EXPORT_SET "ComponentsTargets" CACHE STRING "Export set name used by exported targets")
endif()

# Set C standard
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Set build type if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -fPIC")
set(CMAKE_C_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_RELEASE "-O2")

# Find libgpiod
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBGPIOD libgpiod)

if(NOT LIBGPIOD_FOUND)
    message(WARNING "libgpiod not found. Please install it: sudo apt-get install libgpiod-dev")
endif()

# 获取版本号并传递给编译器
if(LIBGPIOD_VERSION VERSION_LESS "2.0.0")
    add_definitions(-DLIBGPIOD_V1)
else()
    add_definitions(-DLIBGPIOD_V2)
endif()


# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
if(LIBGPIOD_FOUND)
    include_directories(${LIBGPIOD_INCLUDE_DIRS})
endif()

# Source files
set(KEY_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/key.c
)

# Create shared library
add_library(key SHARED ${KEY_SOURCES})

# Link libgpiod and pthread to the shared library
if(LIBGPIOD_FOUND)
    target_link_libraries(key PRIVATE ${LIBGPIOD_LIBRARIES})
endif()
target_link_libraries(key PRIVATE pthread)

# Versioning (output dir should follow the build tree; avoid writing artifacts into source tree)
set_target_properties(key PROPERTIES
    SOVERSION 1
    VERSION 1.0.0
)

# Set public header
target_include_directories(key PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Install rules (so ROS2 nodes can link via PREFIX)
install(TARGETS key
    EXPORT ${SROBOTIS_EXPORT_SET}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(FILES include/key.h DESTINATION include)

# Optional: Build example executable
enable_testing()




# Build example - compile with library and include key.c directly for test mode
add_executable(test_key ${CMAKE_CURRENT_SOURCE_DIR}/example/test_key.c ${CMAKE_CURRENT_SOURCE_DIR}/src/key.c)
target_compile_definitions(test_key PRIVATE KEY_TEST_MODE)
target_include_directories(test_key PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
if(LIBGPIOD_FOUND)
    target_include_directories(test_key PRIVATE ${LIBGPIOD_INCLUDE_DIRS})
    target_link_libraries(test_key PRIVATE ${LIBGPIOD_LIBRARIES})
endif()
target_link_libraries(test_key PRIVATE pthread)
set_target_properties(test_key PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
install(TARGETS test_key RUNTIME DESTINATION bin)

# Add test
add_test(NAME test_key COMMAND test_key)
