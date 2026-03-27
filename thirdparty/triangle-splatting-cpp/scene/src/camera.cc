#include "camera.h"

torch::Tensor utils::getViewMatrix(const Eigen::Quaternionf &q_, const Eigen::Vector3f &t_)
{
  // 1. Convert Quaternion to 3x3 Rotation Matrix
  Eigen::Matrix3f R = q_.toRotationMatrix();

  // 2. Create the 4x4 Transformation Matrix (Rt)
  // We use RowMajor to align with how PyTorch tensors are typically stored
  Eigen::Matrix<float, 4, 4, Eigen::RowMajor> Rt = Eigen::Matrix4f::Identity();

  // Rt[:3, :3] = R.transpose()
  Rt.block<3, 3>(0, 0) = R.transpose();

  // Rt[:3, 3] = t
  Rt.block<3, 1>(0, 3) = t_;

  // 3. Convert Eigen matrix to torch::Tensor
  // We clone() to ensure the tensor owns its memory after the Eigen local variable is destroyed
  return torch::from_blob(Rt.data(), {4, 4}, torch::kFloat32).clone();
}

torch::Tensor utils::getProjectionMatrix(float znear, float zfar, float fovX, float fovY)
{
  // 1. Calculate tangent of half-FOVs
  float tanHalfFovY = std::tan(fovY / 2.0f);
  float tanHalfFovX = std::tan(fovX / 2.0f);

  // 2. Calculate frustum boundaries
  float top = tanHalfFovY * znear;
  float bottom = -top;
  float right = tanHalfFovX * znear;
  float left = -right;

  // 3. Initialize 4x4 zero tensor
  // Using kFloat32 to match your Python float32 logic
  torch::Tensor P = torch::zeros({4, 4}, torch::kFloat32);

  float z_sign = 1.0f;

  // 4. Fill the Projection Matrix
  // Accessing via accessor for performance or index for readability
  P[0][0] = 2.0f * znear / (right - left);
  P[1][1] = 2.0f * znear / (top - bottom);
  P[0][2] = (right + left) / (right - left);
  P[1][2] = (top + bottom) / (top - bottom);
  P[3][2] = z_sign;
  P[2][2] = z_sign * zfar / (zfar - znear);
  P[2][3] = -(zfar * znear) / (zfar - znear);

  return P;
}

Camera::Camera(vk_PinholeCamera_SharedPtr pinhole_camera, const float near, const float far)
  : height_(pinhole_camera->height()),
    width_(pinhole_camera->width()),
    fovx_(2.0 * atan((0.5 * pinhole_camera->width()) / pinhole_camera->fx())),
    fovy_(2.0 * atan((0.5 * pinhole_camera->height()) / pinhole_camera->fy())),
    near_(near),
    far_(far),
    mtx_(std::make_shared<std::shared_mutex>()),
    q_(Eigen::Quaternion<float>::Identity()),
    t_(Eigen::Matrix<float, 3, 1>::Zero())
{
  viewmatrix_ = torch::eye(4).cuda(); // T^C_W
  projmatrix_ = utils::getProjectionMatrix(near_, far_, fovx_, fovy_).transpose(0,1).cuda();
  full_viewmatrix_ = viewmatrix_.cuda().unsqueeze(0).bmm(projmatrix_.unsqueeze(0)).squeeze(0);
}

void Camera::setPose(const Eigen::Quaternion<float>& q, const Eigen::Matrix<float, 3, 1>& t) // q: R^w_c, t: R^c_w @ -t^w
{
  std::unique_lock<std::shared_mutex> lock(*mtx_);
  q_ = q;
  t_ = t;
  viewmatrix_ = utils::getViewMatrix(q_, t_).transpose(0,1).cuda();
  full_viewmatrix_ = viewmatrix_.cuda().unsqueeze(0).bmm(projmatrix_.unsqueeze(0)).squeeze(0);
}