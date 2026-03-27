#include "diff_triangle_rasterization.h"

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
                                                 )
{
  return _RasterizeTriangles::apply(
    triangles_points,
    sigma,
    num_points_per_triangle,
    cumsum_of_points_per_triangle,
    number_of_points,
    sh,
    colors_precomp,
    opacities,
    means2D,
    scaling,
    density_factor,
    raster_settings
  );
}

torch::autograd::tensor_list _RasterizeTriangles::forward(torch::autograd::AutogradContext* ctx,
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
                                                          )
{
  // std::tuple<int, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor, torch::Tensor>
  auto [num_rendered, color, depth, radii, geomBuffer, binningBuffer, imgBuffer, scaling_, density_factor_, max_blending] = 
    RasterizetrianglesCUDA(
        raster_settings.bg,
        triangles_points,
        sigma,
        num_points_per_triangle,
        cumsum_of_points_per_triangle,
        colors_precomp,
        opacities,
        scaling,
        density_factor,
        raster_settings.viewmatrix,
        raster_settings.projmatrix,
        number_of_points,
        raster_settings.tanfovx,
        raster_settings.tanfovy,
        raster_settings.image_height,
        raster_settings.image_width,
        sh,
        raster_settings.sh_degree,
        raster_settings.campos,
        raster_settings.prefiltered,
        false
    );
  ctx->save_for_backward({triangles_points, sigma, num_points_per_triangle, cumsum_of_points_per_triangle, colors_precomp, radii, sh, geomBuffer, binningBuffer, imgBuffer});
  ctx->saved_data["num_rendered"] = num_rendered;
  ctx->saved_data["number_of_points"] = number_of_points;
  ctx->saved_data["background"] = raster_settings.bg;
  ctx->saved_data["viewmatrix"] = raster_settings.viewmatrix;
  ctx->saved_data["projmatrix"] = raster_settings.projmatrix;
  ctx->saved_data["tanfovx"] = raster_settings.tanfovx;
  ctx->saved_data["tanfovy"] = raster_settings.tanfovy;
  ctx->saved_data["sh_degree"] = raster_settings.sh_degree;
  ctx->saved_data["campos"] = raster_settings.campos;
  return {color, radii, scaling_, density_factor_, depth, max_blending};
}

torch::autograd::tensor_list _RasterizeTriangles::backward(torch::autograd::AutogradContext* ctx, torch::autograd::tensor_list grad_outputs) 
{
  auto grad_out_color = grad_outputs[0];
  auto grad_out_depth = grad_outputs[4];
  auto saved_vars = ctx->get_saved_variables();  
  
  auto triangles_points = saved_vars[0];  
  auto sigma = saved_vars[1];  
  auto num_points_per_triangle = saved_vars[2];  
  auto cumsum_of_points_per_triangle = saved_vars[3];  
  auto colors_precomp = saved_vars[4];  
  auto radii = saved_vars[5];  
  auto sh = saved_vars[6];  
  auto geomBuffer = saved_vars[7];  
  auto binningBuffer = saved_vars[8];  
  auto imgBuffer = saved_vars[9];
  
  auto [grad_triangles, grad_sigma, grad_colors_precomp, grad_opacities, grad_sh, grad_means2D] = 
    RasterizetrianglesBackwardCUDA(
        ctx->saved_data["background"].to<torch::Tensor>(),
        triangles_points,
        sigma,
        num_points_per_triangle,
        cumsum_of_points_per_triangle,
        radii, 
        colors_precomp, 
        ctx->saved_data["viewmatrix"].to<torch::Tensor>(), 
        ctx->saved_data["projmatrix"].to<torch::Tensor>(), 
        ctx->saved_data["number_of_points"].to<int>(),
        ctx->saved_data["tanfovx"].to<float>(),
        ctx->saved_data["tanfovy"].to<float>(),
        grad_out_color, 
        grad_out_depth,
        sh, 
        ctx->saved_data["sh_degree"].to<int>(),
        ctx->saved_data["campos"].to<torch::Tensor>(),
        geomBuffer,
        ctx->saved_data["num_rendered"].to<int>(),
        binningBuffer,
        imgBuffer,
        false
    );

  auto grad_triangles_flatten = grad_triangles.flatten(0);
  auto grad_sigma_view = grad_sigma.view({-1, 1});

  return 
  {
    grad_triangles_flatten, 
    grad_sigma_view,
    torch::Tensor(),
    torch::Tensor(),
    torch::Tensor(),
    grad_sh,
    grad_colors_precomp,
    grad_opacities,
    grad_means2D,
    torch::Tensor(),
    torch::Tensor(),
    torch::Tensor()
  };
}

TriangleRasterizer::TriangleRasterizer(const TriangleSplattingSettings& raster_settings)
 : torch::nn::Module(), raster_settings_(raster_settings) {}

torch::Tensor TriangleRasterizer::mark_visible(torch::Tensor triangles_points)
{
  torch::NoGradGuard no_grad;
  auto visible = markVisible(triangles_points, raster_settings_.viewmatrix, raster_settings_.projmatrix);
  return visible;
}

torch::autograd::tensor_list TriangleRasterizer::forward(const torch::Tensor& triangles_points,
                                                         const torch::Tensor& sigma,
                                                         const torch::Tensor& num_points_per_triangle,
	                                                       const torch::Tensor& cumsum_of_points_per_triangle,
                                                         const int number_of_points,
                                                         const torch::Tensor& opacities,
                                                         const torch::Tensor& means2D,
                                                         torch::Tensor& scaling,
                                                         torch::Tensor& density_factor,
                                                         const torch::Tensor& sh,
                                                         const torch::Tensor& colors_precomp)
{
  if ((sh.defined() && colors_precomp.defined()) || (!sh.defined() && !colors_precomp.defined())) 
    throw std::invalid_argument("Please provide exactly one of either SH or precomputed colors!");

  return rasterize_triangles(
    triangles_points,
    sigma,
    num_points_per_triangle,
    cumsum_of_points_per_triangle,
    number_of_points,
    sh,
    colors_precomp,
    opacities,
    means2D,
    scaling,
    density_factor,
    raster_settings_
  );
}