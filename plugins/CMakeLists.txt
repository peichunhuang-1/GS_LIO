file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *)
set(PLUGIN_TARGETS "")
foreach(child ${children})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
        add_subdirectory(${child})
    endif()
endforeach()

add_library(plugins INTERFACE)

target_link_libraries(plugins INTERFACE
  -Wl,--whole-archive
  ${PLUGIN_TARGETS}
  -Wl,--no-whole-archive
)