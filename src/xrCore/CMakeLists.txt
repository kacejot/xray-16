project(xrCore)

list(APPEND DIRS
    "."
    "Animation"
    "Compression"
    "Containers"
    "Crypto"
    "Math"
    "Media"
    "Memory"
    "PostProcess"
    "Text"
    "Threading"
    "XML"
    "../xrCommon"
    )

add_dir("${DIRS}")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Externals/pugixml/src
    ${SDL_INCLUDE_DIRS}
    ${TBB_INCLUDE_DIRS}
)

list(REMOVE_ITEM ${PROJECT_NAME}__SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/Memory/xrMemory_align.cpp")
list(REMOVE_ITEM ${PROJECT_NAME}__INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}/Memory/xrMemory_align.h")

add_definitions(-DXRCORE_EXPORTS -DGIT_INFO_CURRENT_COMMIT=${GIT_COMMIT} -DGIT_INFO_CURRENT_BRANCH=${GIT_BRANCH})
add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}__SOURCES} ${${PROJECT_NAME}__INCLUDES})

set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_link_libraries(${PROJECT_NAME} xrMiscMath dl pthread ${PCRE_LIBRARIES} ${SDL_LIBRARIES} ${LZO_LIBRARIES} ${CRYPTO++_LIBRARIES} ${PUGIXML_LIBRARY} ${TBB_LIBRARIES})
xr_install(${PROJECT_NAME})
