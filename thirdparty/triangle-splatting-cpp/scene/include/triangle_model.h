#pragma once

#include <torch/torch.h>
#include <torch/script.h>
#include "camera.h"
#include <pcl_conversions/pcl_conversions.h>
#include <torch_delaunay/predicate.h>
#include <torch_delaunay/sweephull.h>
#include <torch_delaunay/triangle.h>

std::vector<char> get_bytes_from_file(const std::string& filename);


class TriangleModel
{
  public:
    TriangleModel();
    void setup_optimizer(double lr=0.001);
    void load(const std::string& path); // load from pretrained model
    void start_from_pcd_and_keyframe(const pcl::PointCloud<pcl::PointXYZ> &pcd, const Camera &camera, const cv::Mat &keyframe);
    // properties
    inline torch::Tensor get_triangle_points() const {return _triangles_points;}
    inline torch::Tensor get_opacity() const {return torch::sigmoid(_opacity);}
    inline torch::Tensor get_sigma() const { return 0.01 + torch::exp(_sigma);}
    inline torch::Tensor get_triangles_points_flatten() const {return _triangles_points.flatten(0);}
    inline torch::Tensor get_sh() const 
    {
      auto features_dc = _features_dc;
      auto features_rest = _features_rest;
      return torch::cat({features_dc, features_rest}, 1);
    }
    inline torch::Tensor get_num_points_per_triangle() const {return _num_points_per_triangle;}
    inline torch::Tensor get_cumsum_of_points_per_triangle() const {return _cumsum_of_points_per_triangle;}
    inline int get_number_of_points() const {return _number_of_points;}
    inline int sh_degree() const {return active_sh_degree;}
    void step() 
    {
      _optimizer->step(); 
      _optimizer->zero_grad(true);
    }
    void extend_from_pcd(at::Tensor new_triangles, at::Tensor new_feature_dc, const int sh_degree);
  private:
    // utility function
    void project_pcd_on_frame(const pcl::PointCloud<pcl::PointXYZ> &pcd, const Camera &camera, const cv::Mat &keyframe, torch::Tensor &triangles, torch::Tensor &features_dc);
    at::Tensor pcl_to_tensor(const pcl::PointCloud<pcl::PointXYZ> &pcd);
    // properties member
    torch::Tensor _triangles_points;
    torch::Tensor _opacity;
    torch::Tensor _sigma;
    torch::Tensor _features_dc;
    torch::Tensor _features_rest;
    torch::Tensor _mask;
    torch::Tensor _num_points_per_triangle;
    torch::Tensor _cumsum_of_points_per_triangle;
    int active_sh_degree;
    int _number_of_points;
    std::unique_ptr<torch::optim::Adam> _optimizer;
};
