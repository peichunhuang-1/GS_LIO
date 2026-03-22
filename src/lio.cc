#include "lio.h"

namespace gs_lio
{
  
Lio::Lio(const std::string &name) : Imu(name)
{
  std::string lidar_plugin_name = "livox";
  this->declare_parameter<std::string>("lio.lidar_plugin_name", "livox");
  this->get_parameter_or<std::string>("lio.lidar_plugin_name", lidar_plugin_name, "livox");
  lidar = LidarPluginFactory::instance().create(lidar_plugin_name, *this);
  if (lidar == nullptr) throw std::runtime_error("Unknown lidar plugin name, ensure you build the plugin and registed");
  
  this->declare_parameter<int>("lio.max_iter", 5);
  this->get_parameter_or<int>("lio.max_iter", lio_max_iter, 5);
  this->declare_parameter<int>("lio.radius_sigma_num", 5);
  this->get_parameter_or<int>("lio.radius_sigma_num", radius_sigma_num, 5);
  this->declare_parameter<int>("lio.distance_sigma_num", 3);
  this->get_parameter_or<int>("lio.distance_sigma_num", distance_sigma_num, 3);

  this->declare_parameter<std::string>("lio.lidar_link", "lidar_link");
  this->get_parameter_or<std::string>("lio.lidar_link", lidar_link, "lidar_link");
  this->declare_parameter<std::string>("lio.imu_link", "imu_link");
  this->get_parameter_or<std::string>("lio.imu_link", imu_link, "imu_link");

  tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  voxel_tree = std::make_shared<VoxelOctoTree>(*this);
}

Lio::~Lio()
{
  if (build_map_thread.joinable()) build_map_thread.join();
}

bool Lio::try_initialize() 
{
  if (current_status.load() == INITIALED) return true;
  if (current_status.load() == IDLE) 
  {
    bool success = tf_buffer->canTransform(
      imu_link,   // target
      lidar_link,     // source
      tf2::TimePointZero,
      tf2::durationFromSec(3.0)
    );
    if (!success) return false;

    auto T_imu_lidar = tf_buffer->lookupTransform(
        imu_link,
        lidar_link,
        tf2::TimePointZero
    );
    auto &t = T_imu_lidar.transform.translation;
    auto &q = T_imu_lidar.transform.rotation;
    lidar_imu_extrinsic_translation = vector3_t(t.x, t.y, t.z);
    lidar_imu_extrinsic_orientation = quaternion_t(q.w, q.x, q.y, q.z);
    current_status = READY;
  }
  RCLCPP_WARN(this->get_logger(), "Initializing, do not move sensors, current valid plane %d", VoxelOctoTree::VALID_PLANE_NUM.load());
  propagated_queue.clear();
  auto raw_points = lidar->get_pointcloud();
  pcl::PointCloud<pcl::PointXYZITC> projected_undistorted_pointcloud;
  projected_undistorted_pointcloud.points.reserve(raw_points->points.size());
  for (int i = 0; i < raw_points->points.size(); i++) 
  {
    const auto &p = raw_points->points[i];
    pcl::PointXYZITC projected_point_with_cov;
    projected_point_with_cov.xyz() = p.xyz();
    projected_point_with_cov.intensity = p.intensity;
    projected_point_with_cov.timestamp = p.timestamp;
    projected_point_with_cov.covariance() = lidar->point_covariance(p.xyz().cast<scalar_t>()).cast<float>();
    projected_undistorted_pointcloud.points.push_back(projected_point_with_cov);
  }
  auto world_points = transform_pointcloud_to_world_frame(
      projected_undistorted_pointcloud,
      get_state());
  voxel_tree->UpdateVoxelOctoTree(world_points);
  if (VoxelOctoTree::VALID_PLANE_NUM > valid_plane_threshold) 
  {
    RCLCPP_INFO(this->get_logger(), "Lio initialized");
    current_status.store(INITIALED);
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZITC> Lio::undistorted_pointcloud(const state_t &current_state, const std::shared_ptr<pcl::PointCloud<pcl::PointXYZIT>> &raw_pointcloud) 
{
  pcl::PointCloud<pcl::PointXYZITC> projected_undistorted_pointcloud;
  if (propagated_queue.empty()) return std::move(projected_undistorted_pointcloud);
  projected_undistorted_pointcloud.points.reserve(raw_pointcloud->points.size());
  state_t cptr = propagated_queue.front();
  int point_index = 0;
  static matrix3_t lidar_imu_extrinsic_orientation_matrix = lidar_imu_extrinsic_orientation.toRotationMatrix();
  while (point_index < raw_pointcloud->points.size()) {
    propagated_queue.pop_front();
    if (propagated_queue.empty()) break;
    for (int i = point_index; i < raw_pointcloud->points.size(); i++) {
      const auto &p = raw_pointcloud->points[i];
      double tailstamp = p.timestamp;
      point_index = i;
      if (tailstamp > propagated_queue.front().get_timestamp()) {
        break;
      }
      if (tailstamp < cptr.get_timestamp()) {
        continue;
      }
      state_t measured_state = cptr.slerp(tailstamp);
      vector3_t projected_point = current_state.measurement_project(
        measured_state,
        p.xyz().cast<scalar_t>(),
        lidar_imu_extrinsic_orientation_matrix,
        lidar_imu_extrinsic_translation
      );
      pcl::PointXYZITC projected_point_with_cov;
      projected_point_with_cov.x = projected_point.x();
      projected_point_with_cov.y = projected_point.y();
      projected_point_with_cov.z = projected_point.z();
      projected_point_with_cov.intensity = p.intensity;
      projected_point_with_cov.timestamp = p.timestamp;
      projected_point_with_cov.covariance() = lidar->point_covariance(projected_point).cast<float>();
      projected_undistorted_pointcloud.points.push_back(projected_point_with_cov);
    }
    cptr = propagated_queue.front();
  }
  return std::move(projected_undistorted_pointcloud);
}

pcl::PointCloud<pcl::PointXYZITC> Lio::transform_pointcloud_to_world_frame(
  const pcl::PointCloud<pcl::PointXYZITC> &pointcloud,
  const state_t &state) {
  pcl::PointCloud<pcl::PointXYZITC> pointcloud_world;
  pointcloud_world.points.resize(pointcloud.points.size());
  static matrix3_t lidar_imu_extrinsic_orientation_matrix = lidar_imu_extrinsic_orientation.toRotationMatrix();
  #pragma omp parallel for num_threads(8)
  for (int i = 0; i < pointcloud.points.size(); i++) {
    const auto &p = pointcloud.points[i];
    vector3_t p_lidar = p.xyz().cast<scalar_t>();
    vector3_t p_body = lidar_imu_extrinsic_orientation_matrix * p_lidar + lidar_imu_extrinsic_translation;
    vector3_t p_world = state.get_rotation() * p_body + state.get_translation();
    pcl::PointXYZITC &pw = pointcloud_world.points[i];
    pw.xyz() = p_world.cast<float>();
    pw.intensity = p.intensity;
    pw.timestamp = p.timestamp;
    matrix3_t point_cross = Sophus::SO3<scalar_t>::hat(p_body);
    
    pw.covariance() = (state.get_rotation().matrix() * p.covariance().cast<scalar_t>() * state.get_rotation().matrix().transpose() + 
                       point_cross * state.get_covariance().block<3, 3>(0, 0) * point_cross.transpose() + 
                       state.get_covariance().block<3, 3>(3, 3)).cast<float>();
  }
  return pointcloud_world;
}

std::shared_ptr<Residual> Lio::build_residual(const state_t& current_state, const pcl::PointXYZITC &pw, const pcl::PointXYZITC &pl, const std::shared_ptr<PlaneImpl> &plane) {
  std::shared_ptr<Residual> residual = std::make_shared<Residual>();
  vector3_t point_to_center_vector = pw.xyz().cast<scalar_t>() - plane->center();
  scalar_t point_to_plane_distance = plane->normal().dot(pw.xyz().cast<scalar_t>()) + plane->d();
  scalar_t point_to_center_distance = point_to_center_vector.norm();
  scalar_t range_distance = sqrt(point_to_center_distance * point_to_center_distance - point_to_plane_distance * point_to_plane_distance);
  static matrix3_t lidar_imu_extrinsic_orientation_matrix = lidar_imu_extrinsic_orientation.toRotationMatrix();
  if (range_distance <= static_cast<scalar_t>(radius_sigma_num * plane->radius())) {
    matrix_t<1, 6> J_nq;
    J_nq.block<1, 3>(0, 0) = point_to_center_vector;
    J_nq.block<1, 3>(0, 3) = -plane->normal();
    scalar_t res = (J_nq * plane->uncertainty() * J_nq.transpose()).value();
    res += (plane->normal().transpose() * pw.covariance().cast<scalar_t>() * plane->normal()).value();
    if (fabs(point_to_plane_distance) < static_cast<scalar_t>(distance_sigma_num * sqrt(res))) {
      residual->distance_to_plane = static_cast<scalar_t>(point_to_plane_distance);
      residual->residual = 1e-3 + static_cast<scalar_t>(res);
      vector3_t p_lidar = pl.xyz().cast<scalar_t>();
      vector3_t p_body = lidar_imu_extrinsic_orientation_matrix * p_lidar + lidar_imu_extrinsic_translation;
      matrix3_t point_cross = Sophus::SO3<scalar_t>::hat(p_body);
      vector3_t A = point_cross * current_state.get_rotation().matrix().transpose() * plane->normal().cast<scalar_t>();
      residual->H_row << A(0), A(1), A(2), plane->normal()(0), plane->normal()(1), plane->normal()(2);
      return residual;
    } else return nullptr;
  } else return nullptr;
}

vector18_t Lio::ieskf(const std::vector<std::shared_ptr<Residual>> &residuals, 
                    const vector18_t &error_state){
  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();
  int valid_observation = residuals.size();
  Eigen::Matrix<scalar_t, Eigen::Dynamic, 6> H(valid_observation, 6);
  Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> H_T_residual_inv(6, valid_observation);
  Eigen::Vector<scalar_t, Eigen::Dynamic> residual_inv(valid_observation);
  Eigen::Vector<scalar_t, Eigen::Dynamic> measurements(valid_observation);
  measurements.setZero();

  for (int i = 0; i < valid_observation; i++) {
    H.row(i) = residuals[i]->H_row;
    residual_inv(i) = 1.0 / residuals[i]->residual;
    H_T_residual_inv.col(i) = residual_inv(i) * residuals[i]->H_row;
    measurements(i) = -residuals[i]->distance_to_plane;
  }
  Eigen::Matrix<scalar_t, 6, 1> H_T_measurements;
  H_T_measurements.noalias() = H_T_residual_inv * measurements;
  H_T_H.block<6, 6>(0, 0) = H_T_residual_inv * H;
  matrix18_t K;
  K.noalias() = (H_T_H.block<18, 18>(0, 0) + get_state().get_covariance().block<18, 18>(0, 0).inverse()).inverse();
  G.block<18, 6>(0, 0) = K.block<18, 6>(0, 0) * H_T_H.block<6, 6>(0, 0);
  return K.block<18, 6>(0, 0) * H_T_measurements + error_state.segment<18>(0) - G.block<18, 6>(0, 0) * error_state.segment<6>(0);
}


void Lio::optimize() {
  if (!try_initialize()) return;
  
  state_t propagated_state = get_state();
  auto raw_points = lidar->get_pointcloud();
  auto projected_undistorted_points = undistorted_pointcloud(propagated_state, raw_points);

  state_t updated_state = propagated_state;
  for (int i = 0; i < lio_max_iter; i++) {
    auto world_points = transform_pointcloud_to_world_frame(
      projected_undistorted_points,
      updated_state);
    int valid_observation = 0;
    std::vector<std::shared_ptr<Residual>> residuals;
    residuals.reserve(world_points.points.size());
    for (int i = 0; i < world_points.points.size(); i++) {
      auto plane = voxel_tree->GetPlane(world_points.points[i]);
      if (plane) {
        auto residual = build_residual(updated_state, world_points.points[i], projected_undistorted_points.points[i], plane);
        if (!residual) continue;
        residuals.push_back(residual);
        valid_observation++;
      }
    }
    if (valid_observation < 100) {
      RCLCPP_WARN(this->get_logger(), "No enough point to optimize, valid number %d", valid_observation);
      propagated_queue.clear();
      return;
    }

    vector18_t error_state = propagated_state - updated_state;
    vector18_t compensated_state = ieskf(residuals, error_state);

    updated_state += compensated_state;
    set_state(updated_state); // sync the state with optimal state
    
    if ((compensated_state.block<3, 1>(0, 0).norm() * 57.3 < 0.01) && 
    (compensated_state.block<3, 1>(3, 0).norm() * 100 < 0.015)) break; // converged, early break
  }

  // update covariance
  updated_state.set_covariance( (I_STATE - G) * updated_state.get_covariance() );
  set_state(updated_state);

  auto world_points = transform_pointcloud_to_world_frame(
      projected_undistorted_points,
      get_state());
    
  if (build_map_thread.joinable()) build_map_thread.join();

  build_map_thread = std::thread([this, world_points]() {
    this->voxel_tree->UpdateVoxelOctoTree(world_points);
  });
  propagated_queue.clear();
}

void Lio::reset(const state_t &state) 
{
  current_status.store(IDLE);
  if (build_map_thread.joinable()) build_map_thread.join();
  voxel_tree.reset(new VoxelOctoTree(*this));
  estimator::reset(state);
}

stamp_t Lio::wait_lidar(int timeout_ms)
{
  return lidar->wait_available(timeout_ms);
}

}

#include "imu.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto lio_node = std::make_shared<gs_lio::Lio>("lio_node");
  auto thread_ = std::thread([lio_node](){
    while (rclcpp::ok())
    {
      auto tailstamp = lio_node->wait_lidar();
      auto t_start = std::chrono::steady_clock::now();
      if (tailstamp < 0) continue;
      if (!lio_node->is_init()) tailstamp = -1;
      while (!lio_node->forward(tailstamp) && tailstamp > 0) {}
      lio_node->optimize();
      auto t_end = std::chrono::steady_clock::now();
      double ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
      RCLCPP_INFO(lio_node->get_logger(), "elapse time: %.3f ms", ms);
    }
  });
  rclcpp::spin(lio_node);
  rclcpp::shutdown();
  return 0;
}