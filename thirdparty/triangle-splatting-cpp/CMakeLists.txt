set(LIBTORCH_DIR "/usr/local/libtorch")
set(CMAKE_PREFIX_PATH ${LIBTORCH_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_compile_options(-fPIC)

find_package(Torch REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${Python3_INCLUDE_DIRS} $<INSTALL_INTERFACE:include>)

add_subdirectory(submodules/gDel2D)
add_subdirectory(submodules/diff-triangle-rasterization)
add_subdirectory(submodules/simple-knn)
add_subdirectory(submodules/vikit_common)

add_library(render scene/src/camera.cc scene/src/triangle_model.cc scene/src/render.cc)
target_include_directories(render PUBLIC scene/include ${TORCH_INCLUDE_DIRS})
target_link_libraries(render 
                      DiffRasterization 
                      ${TORCH_LIBRARIES}
                      Threads::Threads
                      ${OpenCV_LIBRARIES}
                      Python3::Python
                      CUDA::cudart
                      CUDA::curand
                      CUDA::cublas
                      simple-knn
                      mimalloc-static
                      vikit)

ament_target_dependencies(render
  pcl_conversions
)

target_compile_options(render PRIVATE -O3 -use_fast_math)

add_library(utils utils/src/loss.cc)
target_include_directories(utils PUBLIC utils/include ${TORCH_INCLUDE_DIRS})
target_link_libraries(utils 
                      ${TORCH_LIBRARIES}
                      Python3::Python
                      CUDA::cudart
                      CUDA::curand
                      CUDA::cublas)
target_compile_options(utils PRIVATE -O3 -use_fast_math)

add_executable(renderer scene/src/trainer.cc)
target_link_libraries(renderer render utils)
target_compile_options(renderer PRIVATE -O3 -use_fast_math)