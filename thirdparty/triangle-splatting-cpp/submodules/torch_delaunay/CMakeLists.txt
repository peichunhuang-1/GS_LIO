set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
set(torch_delaunay_LIBRARY_VERSION "1.3.0")
set(torch_delaunay_LIBRARY_SOVERSION "1")

aux_source_directory(src torch_delaunay_SOURCES)

add_library(torch_delaunay SHARED ${torch_delaunay_SOURCES})
add_compile_options(-O3 -DNDEBUG)
target_link_libraries(torch_delaunay "${TORCH_LIBRARIES}")
target_include_directories(torch_delaunay PUBLIC include)