#pragma once

#include <torch/torch.h>

namespace loss_utils
{

torch::Tensor equilateral_regularizer(const torch::Tensor& triangles);

torch::Tensor l1_loss(const torch::Tensor& network_output,
                      const torch::Tensor& gt);

torch::Tensor l2_loss(const torch::Tensor& network_output,
                      const torch::Tensor& gt);

torch::Tensor gaussian_kernel(const int window_size, const float sigma);

torch::Tensor create_window(const int window_size, const int channel);

torch::Tensor ssim_loss(const torch::Tensor& network_output,
                        const torch::Tensor& gt,
                        const int window_size = 11,
                        const bool size_average = true);

torch::Tensor _ssim_loss(const torch::Tensor& network_output,
                         const torch::Tensor& gt,
                         const torch::Tensor & window,
                         const int window_size,
                         const int channel,
                         const bool size_average);

}