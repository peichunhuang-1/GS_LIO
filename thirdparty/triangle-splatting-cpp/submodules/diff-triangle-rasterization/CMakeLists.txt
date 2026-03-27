#
# The original code is under the following copyright:
# Copyright (C) 2023, Inria
# GRAPHDECO research group, https://team.inria.fr/graphdeco
# All rights reserved.
#
# This software is free for non-commercial, research and evaluation use 
# under the terms of the LICENSE_GS.md file.
#
# For inquiries contact george.drettakis@inria.fr
#
# The modifications of the code are under the following copyright:
# Copyright (C) 2024, University of Liege, KAUST and University of Oxford
# TELIM research group, http://www.telecom.ulg.ac.be/
# IVUL research group, https://ivul.kaust.edu.sa/
# VGG research group, https://www.robots.ox.ac.uk/~vgg/
# All rights reserved.
# The modifications are under the LICENSE.md file.
#
# For inquiries contact jan.held@uliege.be
#

add_library(CudaRasterizer
  cuda_rasterizer/backward.h
  cuda_rasterizer/backward.cu
  cuda_rasterizer/forward.h
  cuda_rasterizer/forward.cu
  cuda_rasterizer/auxiliary.h
  cuda_rasterizer/rasterizer_impl.cu
  cuda_rasterizer/rasterizer_impl.h
  cuda_rasterizer/rasterizer.h
  cuda_rasterizer/utils.h
  cuda_rasterizer/utils.cu
)

set_target_properties(CudaRasterizer PROPERTIES 
  CUDA_ARCHITECTURES "70;75;86"
  CMAKE_CXX_STANDARD 17
  CMAKE_CXX_EXTENSIONS OFF
  CMAKE_CUDA_STANDARD 17
  CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}"
) 

target_include_directories(CudaRasterizer PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cuda_rasterizer)
target_include_directories(CudaRasterizer PRIVATE third_party/glm ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

add_library(DiffRasterization
  diff_triangle_rasterization.h
  diff_triangle_rasterization.cu
  rasterize_points.h
  rasterize_points.cu
)

set_target_properties(DiffRasterization PROPERTIES 
  CUDA_ARCHITECTURES "70;75;86"
  CMAKE_CXX_STANDARD 17
  CMAKE_CXX_EXTENSIONS OFF
  CMAKE_CUDA_STANDARD 17
  CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}"
) 

target_include_directories(DiffRasterization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(DiffRasterization PRIVATE third_party/glm ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES} ${TORCH_INCLUDE_DIRS})

target_link_libraries(DiffRasterization CudaRasterizer)