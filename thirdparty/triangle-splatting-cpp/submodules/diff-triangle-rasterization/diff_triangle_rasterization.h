//
// The original code is under the following copyright:
// Copyright (C) 2023, Inria
// GRAPHDECO research group, https://team.inria.fr/graphdeco
// All rights reserved.
//
// This software is free for non-commercial, research and evaluation use 
// under the terms of the LICENSE_GS.md file.
//
// For inquiries contact george.drettakis@inria.fr
//
// The modifications of the code are under the following copyright:
// Copyright (C) 2024, University of Liege, KAUST and University of Oxford
// TELIM research group, http://www.telecom.ulg.ac.be/
// IVUL research group, https://ivul.kaust.edu.sa/
// VGG research group, https://www.robots.ox.ac.uk/~vgg/
// All rights reserved.
// The modifications are under the LICENSE.md file.
//
// For inquiries contact jan.held@uliege.be
//
// The modifications of the code are under the following copyright:
// Copyright (C) 2026, Delta Inc.
// All right reserved.
// The modifications are under the LICENSE_DELTA.md file
// 
// For inquiries contact peichun.huang@deltaww.com

#ifndef DIFF_TRIANGLE_RASTERIZATION_H
#define DIFF_TRIANGLE_RASTERIZATION_H

#include "rasterize_points.h"

struct TriangleSplattingSettings 
{
  int image_height;
  int image_width; 
  float tanfovx;
  float tanfovy;
  torch::Tensor bg;
  torch::Tensor viewmatrix;
  torch::Tensor projmatrix;
  int sh_degree;
  torch::Tensor campos;
  bool prefiltered;
};

torch::autograd::tensor_list rasterize_triangles(const torch::Tensor& triangles_points,
	                                               const torch::Tensor& sigma,
                                                 const torch::Tensor& num_points_per_triangle,
	                                               const torch::Tensor& cumsum_of_points_per_triangle,
                                                 const int number_of_points,
                                                 const torch::Tensor& sh,
                                                 const torch::Tensor& colors_precomp,
                                                 const torch::Tensor& opacities,
                                                 const torch::Tensor& means2D,
                                                 torch::Tensor& scaling,
	                                               torch::Tensor& density_factor,
                                                 const TriangleSplattingSettings& raster_settings
                                                 );

class _RasterizeTriangles: public torch::autograd::Function<_RasterizeTriangles> 
{
  public:
    static torch::autograd::tensor_list forward(torch::autograd::AutogradContext* ctx,
                                                const torch::Tensor& triangles_points,
	                                              const torch::Tensor& sigma,
                                                const torch::Tensor& num_points_per_triangle,
	                                              const torch::Tensor& cumsum_of_points_per_triangle,
                                                const int number_of_points,
                                                const torch::Tensor& sh,
                                                const torch::Tensor& colors_precomp,
                                                const torch::Tensor& opacities,
                                                const torch::Tensor& means2D,
                                                torch::Tensor& scaling,
	                                              torch::Tensor& density_factor,
                                                const TriangleSplattingSettings& raster_settings
                                                );

    static torch::autograd::tensor_list backward(torch::autograd::AutogradContext* ctx, torch::autograd::tensor_list grad_outputs);
};

class TriangleRasterizer : public torch::nn::Module 
{
  public:
    TriangleRasterizer(const TriangleSplattingSettings& raster_settings);
    torch::Tensor mark_visible(torch::Tensor triangles_points);
    torch::autograd::tensor_list forward(const torch::Tensor& triangles_points,
                                         const torch::Tensor& sigma,
                                         const torch::Tensor& num_points_per_triangle,
	                                       const torch::Tensor& cumsum_of_points_per_triangle,
                                         const int number_of_points,
                                         const torch::Tensor& opacities,
                                         const torch::Tensor& means2D,
                                         torch::Tensor& scaling,
                                         torch::Tensor& density_factor,
                                         const torch::Tensor& sh = torch::Tensor(),
                                         const torch::Tensor& colors_precomp = torch::Tensor()
                                        );
  private:
    TriangleSplattingSettings raster_settings_;
};

#endif