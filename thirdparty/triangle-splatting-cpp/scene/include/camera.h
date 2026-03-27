#pragma once

#include <torch/torch.h>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include "vikit/pinhole_camera.h"
#include <Eigen/Dense>
#include <shared_mutex>

using vk_PinholeCamera_SharedPtr = std::shared_ptr<vk::PinholeCamera>;

namespace utils
{
  torch::Tensor getViewMatrix(const Eigen::Quaternionf &q_, const Eigen::Vector3f &t_);
  torch::Tensor getProjectionMatrix(float znear, float zfar, float fovX, float fovY);
}

class Camera: public torch::nn::Module 
{
  public:
    Camera(vk_PinholeCamera_SharedPtr pinhole_camera, const float near, const float far);
    void setPose(const Eigen::Quaternion<float>&q, const Eigen::Matrix<float, 3, 1>& t);
    inline torch::Tensor getViewMatrix() const {
      std::shared_lock<std::shared_mutex> lock(*mtx_);
      return viewmatrix_;
    }
    inline torch::Tensor getProjMatrix() const {
      std::shared_lock<std::shared_mutex> lock(*mtx_);
      return projmatrix_;
    }
    inline torch::Tensor getFullViewMatrix() const {
      std::shared_lock<std::shared_mutex> lock(*mtx_);
      return full_viewmatrix_;
    }
    inline int height() const { return height_; }
    inline int width() const { return width_; }
    inline float fovx() const { return fovx_; }
    inline float fovy() const { return fovy_; }
    inline torch::Tensor get_campos() const {
      std::shared_lock<std::shared_mutex> lock(*mtx_);
      return viewmatrix_.inverse().index({3, torch::indexing::Slice({torch::indexing::None, 3})});
    }
  private:
    const int height_;
    const int width_;
    const float fovx_;
    const float fovy_;
    const float near_;
    const float far_;

    std::shared_ptr<std::shared_mutex> mtx_;
    torch::Tensor viewmatrix_;
    torch::Tensor projmatrix_;
    torch::Tensor full_viewmatrix_;
    Eigen::Quaternion<float> q_;
    Eigen::Matrix<float, 3, 1> t_;
};