project(xrScriptEngine)

list(APPEND DIRS
    "."
    "LuaStudio"
    )

add_dir("${DIRS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Externals/luabind
    ${SDL_INCLUDE_DIRS}
    )

# uncomment next string for debug script engine
#add_definitions(-DCONFIG_SCRIPT_ENGINE_LOG_EXPORTS -DCONFIG_SCRIPT_ENGINE_LOG_SKIPPED_EXPORTS)
add_definitions(-DXRSCRIPTENGINE_EXPORTS)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}__SOURCES} ${${PROJECT_NAME}__INCLUDES})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
target_link_libraries(${PROJECT_NAME} xrAPI xrCore luabind ${LUA_LIBRARIES})
xr_install(${PROJECT_NAME})

