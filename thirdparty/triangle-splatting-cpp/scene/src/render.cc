#include "render.h"

torch::autograd::tensor_list render(
    const Camera &camera,
    const TriangleModel &model
)
{
  auto means2D = torch::zeros_like(model.get_triangle_points().index({"...", 0, "..."}).squeeze()).set_requires_grad(true).to(torch::kCUDA);
  auto scaling = torch::zeros_like(means2D.index({"...", 0})).set_requires_grad(true).to(torch::kCUDA).detach();  
  auto density_factor = torch::zeros_like(scaling).set_requires_grad(true).to(torch::kCUDA).detach();
  means2D.retain_grad();

  auto rasterize_settings = TriangleSplattingSettings {
    camera.height(),
    camera.width(),
    tanf(camera.fovx() * 0.5),
    tanf(camera.fovy() * 0.5),
    torch::tensor({0.f, 0.f, 0.f}, torch::kFloat32).to(torch::kCUDA),
    camera.getViewMatrix(),
    camera.getFullViewMatrix(),
    model.sh_degree(),
    camera.get_campos(),
    false
  };

  auto rasterizer = TriangleRasterizer(rasterize_settings);

  auto opacity = model.get_opacity();
  auto sigma = model.get_sigma();
  auto triangles_points = model.get_triangles_points_flatten();
  auto num_points_per_triangle = model.get_num_points_per_triangle();
  auto cumsum_of_points_per_triangle = model.get_cumsum_of_points_per_triangle();
  auto number_of_points = model.get_number_of_points();
  auto shs = model.get_sh();
  auto mask = ((torch::sigmoid(model.get_opacity()) > 0.01).to(torch::kFloat32) - torch::sigmoid(model.get_opacity())).detach() + torch::sigmoid(model.get_opacity());
  opacity = opacity * mask;

  return rasterizer.forward(
    triangles_points,
    sigma,
    num_points_per_triangle,
    cumsum_of_points_per_triangle,
    number_of_points,
    opacity,
    means2D,
    scaling,
    density_factor,
    shs,
    torch::Tensor()
  );
}