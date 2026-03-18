#include "voxel.h"

namespace gs_lio
{
int Voxel::MAX_POINT_NUM = 250;
int Voxel::MAX_LAYER = 3;
scalar_t Voxel::BASIC_VOXEL_SIZE = 0.5;

int VoxelOctoTree::LRU_MAX_VOXEL_NUM = 70000;

Voxel::Voxel(const vector3_t &c, const scalar_t &ql, const int &l) 
: layer(l), voxel_center(c), quater_length(ql)
{
  operation_mtx = std::make_shared<std::shared_mutex>();
  plane = std::make_shared<PlaneImpl>();
}

std::shared_ptr<Voxel> Voxel::InsertPoint(const pcl::PointXYZITC &point_world)
{
  std::unique_lock<std::shared_mutex> lock(*operation_mtx);
  /*
  working flow:
  1. if the voxel is fixed, return nullptr.
     fixed exist two case: 
     a. the plane is valid and has enough points, which means the plane is constructed successfully and won't be updated anymore.
     b. the plane is invalid but has too many points, in this case the voxel must already be divided to the max layer, and the plane is still invalid, which means the plane can't be constructed successfully but we have no chance to update it anymore.
  2. if the plane is invalid but has tried several times, we devide the voxel and inset the point to the sub-voxel.
  3. if the plane is valid or not tried enough times, insert point and set need_update.
  edge case: if the last point makes the plane from valid to invalid
  */
  if (fixed) return nullptr;
  if (!plane->is_valid() && plane->point_num() >= 0.3 * MAX_POINT_NUM)
  {
    if (layer != MAX_LAYER)
    {
      plane.reset(new Plane()); // release plane memory before divide
      int xyz[3] = {0, 0, 0};
      if (point_world.x > voxel_center(0)) { xyz[0] = 1; }
      if (point_world.y > voxel_center(1)) { xyz[1] = 1; }
      if (point_world.z > voxel_center(2)) { xyz[2] = 1; }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr)
      {
        vector3_t c = voxel_center + vector3_t(
            (2 * xyz[0] - 1) * quater_length,
            (2 * xyz[1] - 1) * quater_length,
            (2 * xyz[2] - 1) * quater_length
        );
        leaves[leafnum] = std::make_shared<Voxel>(c, quater_length / 2.f, layer+1);
      }
      return leaves[leafnum]->InsertPoint(point_world);
    }
  }
  if (plane->point_num() >= MAX_POINT_NUM) { 
    fixed = true; 
    return nullptr;
  }
  need_update.store(true);
  plane->insert_point(point_world);
  return shared_from_this();
}

void Voxel::UpdateVoxel()
{
  if (!need_update.load()) return;
  need_update.store(false);
  std::unique_lock<std::shared_mutex> lock(*operation_mtx);
  plane->update();
}

std::shared_ptr<Plane> Voxel::GetPlaneIter(const pcl::PointXYZITC &point)
{
  const Voxel* current = this;
  while (true) {
    std::shared_lock<std::shared_mutex> lock(*current->operation_mtx);
    if (current->plane->is_valid())
        return std::make_shared<PlaneImpl>(*dynamic_cast<PlaneImpl*>(current->plane.get()));
    if (current->layer >= MAX_LAYER)
        return nullptr;
    int xyz[3] = {0, 0, 0};
    if (point.x > voxel_center(0)) { xyz[0] = 1; }
    if (point.y > voxel_center(1)) { xyz[1] = 1; }
    if (point.z > voxel_center(2)) { xyz[2] = 1; }
    int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
    if (!current->leaves[leafnum])
        return nullptr;
    current = current->leaves[leafnum].get();
  }
}

VoxelOctoTree::VoxelOctoTree(rclcpp::Node &node) : clock_(RCL_ROS_TIME)
{
  operation_mtx = std::make_shared<std::shared_mutex>();
  node.declare_parameter<int>("voxel.lru_max_voxel_num", 70000);
  node.get_parameter_or<int>("voxel.lru_max_voxel_num", VoxelOctoTree::LRU_MAX_VOXEL_NUM, 70000);
  grids = std::make_shared<LRUCache<VOXEL_LOCATION, Voxel>>(VoxelOctoTree::LRU_MAX_VOXEL_NUM);
  node.declare_parameter<int>("voxel.max_point_num", 250);
  node.get_parameter_or<int>("voxel.max_point_num", Voxel::MAX_POINT_NUM, 250);
  node.declare_parameter<int>("voxel.max_layer", 3);
  node.get_parameter_or<int>("voxel.max_layer", Voxel::MAX_LAYER, 3);
  node.declare_parameter<scalar_t>("voxel.basic_voxel_size", 0.5);
  node.get_parameter_or<scalar_t>("voxel.basic_voxel_size", Voxel::BASIC_VOXEL_SIZE, 0.5);

  std::string marker_topic = "/voxel/octotree_markers";
  node.declare_parameter<std::string>("voxel.marker_topic", marker_topic);
  node.get_parameter_or<std::string>("voxel.marker_topic", marker_topic, "/voxel/octotree_markers");
  marker_pub = node.create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);
  initialize_marker();
}

void VoxelOctoTree::initialize_marker()
{
  marker.header.frame_id = "map";
  marker.ns = "voxel_octotree";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = Voxel::BASIC_VOXEL_SIZE;
  marker.scale.y = Voxel::BASIC_VOXEL_SIZE;
  marker.scale.z = Voxel::BASIC_VOXEL_SIZE;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
}

void VoxelOctoTree::publish_voxel_markers()
{
  marker.header.stamp = clock_.now();
  marker.id += 1;
  marker_pub->publish(marker);
  marker.points.clear();
}

VOXEL_LOCATION VoxelOctoTree::get_hash(const pcl::PointXYZITC &point_world)
{
  int64_t loc_xyz[3];
  loc_xyz[0] = static_cast<int64_t>(std::floor(point_world.x / Voxel::BASIC_VOXEL_SIZE));
  loc_xyz[1] = static_cast<int64_t>(std::floor(point_world.y / Voxel::BASIC_VOXEL_SIZE));
  loc_xyz[2] = static_cast<int64_t>(std::floor(point_world.z / Voxel::BASIC_VOXEL_SIZE));
  
  return VOXEL_LOCATION(loc_xyz[0], loc_xyz[1], loc_xyz[2]);
}

void VoxelOctoTree::UpdateVoxelOctoTree(const pcl::PointCloud<pcl::PointXYZITC> &points_world)
{
  for (int i = 0; i < points_world.points.size(); i++)
  {
    VOXEL_LOCATION location = get_hash(points_world.points[i]);
    std::shared_lock<std::shared_mutex> lock(*operation_mtx);
    auto grid_iter = grids->get(location);
    if (!grid_iter) 
    {
      lock.unlock();
      vector3_t c(Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(location.x)+0.5), 
                        Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(location.y)+0.5), 
                        Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(location.z)+0.5));
      std::unique_lock<std::shared_mutex> grid_lock(*operation_mtx);
      grids->put(location, std::make_shared<Voxel>(c, Voxel::BASIC_VOXEL_SIZE / scalar_t(4.0), 0));
      // add rviz visualization
      geometry_msgs::msg::Point p;
      p.x = c(0);
      p.y = c(1);
      p.z = c(2);
      marker.points.push_back(p);
    }
    auto update_voxel = grids->get(location)->InsertPoint(points_world.points[i]);
    if (update_voxel) update_voxels_vector.push_back(update_voxel);
  }
  #pragma omp parallel for num_threads(20)
  for (int i = 0; i < static_cast<int>(update_voxels_vector.size()); ++i)
  {
    update_voxels_vector[i]->UpdateVoxel();
  }
  update_voxels_vector.clear();
  publish_voxel_markers(); // publish after updating all voxels
}

VOXEL_LOCATION VoxelOctoTree::get_near(const VOXEL_LOCATION &basic_voxel_location, const pcl::PointXYZITC &point_world)
{
  VOXEL_LOCATION near_voxel_location = basic_voxel_location;
  vector3_t vc = vector3_t(
    Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(basic_voxel_location.x) + 0.5),
    Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(basic_voxel_location.y) + 0.5),
    Voxel::BASIC_VOXEL_SIZE * (static_cast<scalar_t>(basic_voxel_location.z) + 0.5)
  );
  scalar_t q = Voxel::BASIC_VOXEL_SIZE * 0.25;
  if (point_world.x > vc(0) + q) { near_voxel_location.x = near_voxel_location.x + 1; }
  else if (point_world.x < vc(0) - q) { near_voxel_location.x = near_voxel_location.x - 1; }
  if (point_world.y > vc(1) + q) { near_voxel_location.y = near_voxel_location.y + 1; }
  else if (point_world.y < vc(1) - q) { near_voxel_location.y = near_voxel_location.y - 1; }
  if (point_world.z > vc(2) + q) { near_voxel_location.z = near_voxel_location.z + 1; }
  else if (point_world.z < vc(2) - q) { near_voxel_location.z = near_voxel_location.z - 1; }
  return near_voxel_location;
}

std::shared_ptr<Plane> VoxelOctoTree::GetPlane(const pcl::PointXYZITC &point_world, bool get_near_voxel)
{
  std::shared_lock<std::shared_mutex> lock(*operation_mtx);
  VOXEL_LOCATION basic_voxel_location = get_hash(point_world);
  auto grid_iter = grids->get(basic_voxel_location);
  if (grid_iter) return grid_iter->GetPlaneIter(point_world);
  if (!get_near_voxel) return nullptr;
  VOXEL_LOCATION near_voxel_location = get_near(basic_voxel_location, point_world);
  grid_iter = grids->get(near_voxel_location);
  if (grid_iter) return grid_iter->GetPlaneIter(point_world);
  else return nullptr;
}

}