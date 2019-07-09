cmake_minimum_required(VERSION 3.13)
project(OpenXRay)

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_PREFIX "") # Use DESTDIR
set(CMAKE_SKIP_RPATH TRUE)

option (TBB_PARALLEL "Use tbb::parallel for prarticle and skinning acceleration on SMP." ON)
option (NO_TBB_MALLOC "Use standard memory allocation instead of tbbmalloc" ON)

execute_process(COMMAND git rev-parse --verify HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_COMMIT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)

message( "-- Commit: " ${GIT_COMMIT})

execute_process(COMMAND git rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    OUTPUT_VARIABLE GIT_BRANCH
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)

message( "-- Branch: " ${GIT_BRANCH} )

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
	set(ENV{CCACHE_SLOPPINESS} pch_defines,time_macros)
endif(CCACHE_FOUND)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
		message(FATAL_ERROR "Building with a gcc version less than 7.0 is not supported.")
	endif()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive --std=c++17 -Wno-attributes -pipe -Wl,--no-undefined -fvisibility=hidden")

#set (SANITIZE_FLAGS "-fsanitize=address  -fsanitize=leak -fno-omit-frame-pointer -g -fsanitize=undefined -fno-sanitize=vptr")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${SANITIZE_FLAGS}")
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${SANITIZE_FLAGS}")
set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${SANITIZE_FLAGS}")
set (CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} ${SANITIZE_FLAGS}")

set(LUA_LIBRARIES luajit)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif()

add_definitions ( -msse2 -mfpmath=sse )

if( TBB_PARALLEL )
  add_definitions ( -DUSE_TBB_PARALLEL )
endif()

if( NO_TBB_MALLOC )
  add_definitions ( -DNO_TBB_MALLOC )
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	add_definitions(-DDEBUG -DMIXED)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Og")
	set(LUA_LIBRARIES luajit-debug)
endif()

set(LUA_FOUND true)

add_definitions(-D_MT -D_CPPUNWIND -DPURE_DYNAMIC_CAST -DDECLARE_SPECIALIZATION -DM_NOSTDCONTAINERS_EXT)
add_definitions(-DLUABIND_DYNAMIC_LINK) #self-explanatory
add_definitions(-DdSINGLE) # for ODE

set(LUA_INCLUDE_DIR Externals/LuaJIT/src)

if (NOT WIN32)
find_package(GLEW REQUIRED)
find_package(FreeImage REQUIRED)
find_package(LockFile REQUIRED)
find_package(OpenAL REQUIRED)
find_package(Crypto++ REQUIRED)
find_package(Theora REQUIRED)
find_package(OGG REQUIRED)
find_package(SDL2 REQUIRED)
find_package(LZO REQUIRED)
find_package(JPEG REQUIRED)
find_package(TBB REQUIRED)
find_package(PCRE REQUIRED)
endif()

include_directories(${LUA_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/Common )

add_subdirectory(Externals)
add_subdirectory(src)
