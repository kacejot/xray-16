cmake_minimum_required(VERSION 3.13)
project(OpenXRay)

include(${PROJECT_SOURCE_DIR}/cmake/utils.cmake)

git_status()
message( "-- Commit: " ${GIT_COMMIT})
message( "-- Branch: " ${GIT_BRANCH} )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_INSTALL_PREFIX "") # Use DESTDIR
set(CMAKE_SKIP_RPATH TRUE)
set(LUA_LIBRARIES luajit)
set(LUA_FOUND true)
set(LUA_INCLUDE_DIR Externals/LuaJIT/src)

set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
	set(LUA_LIBRARIES luajit-debug)
endif()

option (TBB_PARALLEL "Use tbb::parallel for prarticle and skinning acceleration on SMP." ON)
option (NO_TBB_MALLOC "Use standard memory allocation instead of tbbmalloc" ON)

add_definitions (-msse2 -mfpmath=sse)

if(TBB_PARALLEL)
  add_definitions(-DUSE_TBB_PARALLEL)
endif()

if(NO_TBB_MALLOC)
  add_definitions(-DNO_TBB_MALLOC)
endif()

add_definitions(-D_MT -D_CPPUNWIND -DPURE_DYNAMIC_CAST -DDECLARE_SPECIALIZATION -DM_NOSTDCONTAINERS_EXT)
add_definitions(-DLUABIND_DYNAMIC_LINK) #self-explanatory
add_definitions(-DdSINGLE) # for ODE

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
