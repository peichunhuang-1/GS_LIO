set(SIMPLE_KNN_SOURCES
  simple_knn.cu
  spatial.cu
)

add_library(simple-knn ${SIMPLE_KNN_SOURCES})

set_target_properties(simple-knn PROPERTIES
  CMAKE_CUDA_ARCHITECTURES 86
  CUDA_STANDARD 17 
  CUDA_STANDARD_REQUIRED ON
  CXX_STANDARD 17 
  CXX_STANDARD_REQUIRED ON
)

target_include_directories(simple-knn 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}
  PRIVATE 
  ${TORCH_INCLUDE_DIRS}
)