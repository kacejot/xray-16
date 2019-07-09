macro(add_dir DIRS)
  foreach(dir ${DIRS})
    include_directories  (${dir} )
    file( GLOB ${dir}__INCLUDES_H ${dir} ${dir}/*.h)
    file( GLOB ${dir}__INCLUDES_HPP ${dir} ${dir}/*.hpp)
    list( APPEND ${PROJECT_NAME}__INCLUDES ${${dir}__INCLUDES_H} ${${dir}__INCLUDES_HPP} )
    file( GLOB ${dir}__SOURCES_CPP ${dir} ${dir}/*.cpp ${dir}/*.cxx)
    file( GLOB ${dir}__SOURCES_C ${dir} ${dir}/*.c)
    list( APPEND ${PROJECT_NAME}__SOURCES ${${dir}__SOURCES_C} ${${dir}__SOURCES_CPP} )
  endforeach()
endmacro()

function(xr_install tgt)
	if(NOT MSVC)
		install(TARGETS ${tgt} DESTINATION "."
			PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
			GROUP_READ GROUP_EXECUTE
			WORLD_READ WORLD_EXECUTE) # chmod 755
	else()
		install(TARGETS ${tgt}
			CONFIGURATIONS Debug
			RUNTIME DESTINATION Debug/
			LIBRARY DESTINATION Debug/)
		install(FILES $<TARGET_PDB_FILE:${tgt}>
			CONFIGURATIONS Debug
			DESTINATION Debug/)
		install(TARGETS ${tgt}
			CONFIGURATIONS Release
			RUNTIME DESTINATION Release/
			LIBRARY DESTINATION Release/)
	endif()
endfunction()

# Use only if install defined outside target directory(like luabind, for example)
function(xr_install_file tgt)
	if(NOT MSVC)
		install(FILES $<TARGET_FILE:${tgt}> DESTINATION "."
			PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
			GROUP_READ GROUP_EXECUTE
			WORLD_READ WORLD_EXECUTE) # chmod 755
	else()
		install(FILES $<TARGET_FILE:${tgt}>
			CONFIGURATIONS Debug
			RUNTIME DESTINATION Debug/)
		install(FILES $<TARGET_PDB_FILE:${tgt}>
			CONFIGURATIONS Debug
			DESTINATION Debug/ )
		install(FILES $<TARGET_FILE:${tgt}>
			CONFIGURATIONS Release
			RUNTIME DESTINATION Release/)
	endif()
endfunction()

