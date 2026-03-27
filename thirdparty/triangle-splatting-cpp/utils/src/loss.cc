#include "loss.h"


namespace loss_utils
{

torch::Tensor equilateral_regularizer(const torch::Tensor& triangles)
{
  auto nan_mask = at::isnan(triangles).any().item<bool>();
  if (nan_mask) throw std::runtime_error("nan value in triangles model");
  auto v0 = triangles.select(1, 1) - triangles.select(1, 0);
  auto v1 = triangles.select(1, 2) - triangles.select(1, 0);
  auto cross = torch::linalg_cross(v0, v1, 1);
  auto area = 0.5 * torch::linalg_vector_norm(cross, 2, {1});
  return area;
}

torch::Tensor l1_loss(const torch::Tensor& network_output,
                      const torch::Tensor& gt)
{
  return torch::abs(network_output - gt).mean();
}

torch::Tensor l2_loss(const torch::Tensor& network_output,
                      const torch::Tensor& gt)
{
  return torch::pow(network_output - gt, 2).mean();
}

torch::Tensor gaussian_kernel(int window_size, float sigma)
{
  int center = window_size / 2;
  auto x = torch::arange(window_size, torch::kFloat32) - center;
  auto gauss = torch::exp(-(x * x) / (2 * sigma * sigma));
  return gauss / gauss.sum();
}

torch::Tensor create_window(int window_size, int channel, torch::Device device)
{
  auto _1D_window = gaussian_kernel(window_size, 1.5).unsqueeze(1);   // [W,1]
  auto _2D_window = _1D_window.mm(_1D_window.t())                      // [W,W]
                                .unsqueeze(0)
                                .unsqueeze(0);                       // [1,1,W,W]
  auto window = _2D_window.expand({channel, 1, window_size, window_size})
                          .contiguous()
                          .to(device);
  return window;
}

torch::Tensor _ssim(
    const torch::Tensor& network_output,
    const torch::Tensor& gt,
    const torch::Tensor& window,
    int window_size,
    int channel,
    bool size_average)
{
  namespace F = torch::nn::functional;
  auto options = F::Conv2dFuncOptions()
                      .padding(window_size / 2)
                      .groups(channel);
  // mean
  auto mu1 = F::conv2d(network_output, window, options);
  auto mu2 = F::conv2d(gt, window, options);
  auto mu1_sq = mu1.pow(2);
  auto mu2_sq = mu2.pow(2);
  auto mu1_mu2 = mu1 * mu2;
  // variance
  auto sigma1_sq = F::conv2d(network_output * network_output, window, options) - mu1_sq;
  auto sigma2_sq = F::conv2d(gt * gt, window, options) - mu2_sq;
  auto sigma12   = F::conv2d(network_output * gt, window, options) - mu1_mu2;
  const float C1 = 0.01f * 0.01f;
  const float C2 = 0.03f * 0.03f;
  auto ssim_map =
      ((2 * mu1_mu2 + C1) * (2 * sigma12 + C2)) /
      ((mu1_sq + mu2_sq + C1) * (sigma1_sq + sigma2_sq + C2));
  if (size_average)
    return ssim_map.mean();
  else
    return ssim_map.mean(1).mean(1).mean(1);
}

torch::Tensor ssim(
    const torch::Tensor& network_output,
    const torch::Tensor& gt,
    int window_size = 11,
    bool size_average = true)
{
  int channel = network_output.size(1);  // N,C,H,W -> C is dim 1
  // create Gaussian window on same device as network_output
  auto window = create_window(window_size, channel, network_output.device());
  window = window.to(network_output.dtype());
  return _ssim(network_output, gt, window, window_size, channel, size_average);
}

} // namespace loss_utils