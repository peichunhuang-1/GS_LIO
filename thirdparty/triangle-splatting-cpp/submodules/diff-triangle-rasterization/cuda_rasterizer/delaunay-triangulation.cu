#include "delaunay-triangulation.h"
#include <cooperative_groups.h>
#include <cooperative_groups/reduce.h>
namespace cg = cooperative_groups;

__device__ static float atomicMax(float* address, float val)
{
    int* address_as_i = (int*) address;
    int old = *address_as_i, assumed;
    do {
        assumed = old;
        old = ::atomicCAS(address_as_i, assumed,
            __float_as_int(::fmaxf(val, __int_as_float(assumed))));
    } while (assumed != old);
    return __int_as_float(old);
}

template<int C> __global__ void project_point(
    const int N,
    const int H,
    const int W,
    const float* pcd,
    const float* viewmatrix,
    const float* projmatrix,
    float* point2d,
    const float min_dist,
    const float max_dist,
    const int grid,
    float* grid_record,
    bool* mask
)
{
    auto idx = cg::this_grid().thread_rank();
	if (idx >= N) return;
    float3 p = {pcd[3 * idx], pcd[3 * idx + 1], pcd[3 * idx + 2]};
    float4 p_hom = transformPoint4x4(p, projmatrix);
	float p_w = 1.0f / (p_hom.w + 0.0000001f);
    float3 p_view = transformPoint4x3(p, viewmatrix);
	float3 p_camera_view = { p_hom.x * p_w, p_hom.y * p_w, p_hom.z * p_w };
	float2 pixel2D = { ndc2Pix(p_camera_view.x, W), ndc2Pix(p_camera_view.y, H) };
    if (p_view.z > max_dist || p_view.z < min_dist 
        || pixel2D.x > W || pixel2D.y > H
        || pixel2D.x < 0 || pixel2D.y < 0) {
        mask[idx] = false;
        return;
    }
    int grid_h = H / grid;
    int grid_w = W / grid;
    int grid_idx = grid_h * W + grid_w;
    if (atomicMax(grid_record[grid_idx], p_view.z) == 0) { // first point
        point2d[2 * idx] = pixel2D.x;
        point2d[2 * idx + 1] = pixel2D.y;
        mask[idx] = true;
    } else {
        mask[idx] = false;
    }
}

template<int C> __global__ void colorize_triangle(
    const int N,
    const int H, 
    const int W,
    const float* pcd,
    const float* point2d,
    const float* image,
    const int* triangles_index,
    float* features_dc,
    bool* mask,
    const float dist_threshold
)
{
    auto idx = cg::this_grid().thread_rank();
	if (idx >= N) return;
    int v0_idx = triangles_index[3 * idx];
    int v1_idx = triangles_index[3 * idx + 1];
    int v2_idx = triangles_index[3 * idx + 2];

    float3 p0 = {pcd[3 * v0_idx], pcd[3 * v0_idx + 1], pcd[3 * v0_idx + 2]};
    float3 p1 = {pcd[3 * v1_idx], pcd[3 * v1_idx + 1], pcd[3 * v1_idx + 2]};
    float3 p2 = {pcd[3 * v2_idx], pcd[3 * v2_idx + 1], pcd[3 * v2_idx + 2]};

    auto dist = [] __device__ (float3 a, float3 b) {
        return sqrtf((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
    };
    if (dist(p0, p1) > dist_threshold || 
        dist(p1, p2) > dist_threshold || 
        dist(p2, p0) > dist_threshold) 
    {
        mask[idx] = false;
        return;
    }
    // colorize the triangle by pixel center
    float2 pix0 = {point2d[2 * v0_idx], point2d[2 * v0_idx + 1]};
    float2 pix1 = {point2d[2 * v1_idx], point2d[2 * v1_idx + 1]};
    float2 pix2 = {point2d[2 * v2_idx], point2d[2 * v2_idx + 1]};

    float center_x = (pix0.x + pix1.x + pix2.x) / 3.0f;
    float center_y = (pix0.y + pix1.y + pix2.y) / 3.0f;

    int x = __float2int_rn(center_x);
    int y = __float2int_rn(center_y);
    if (x < 0 || x >= W || y < 0 || y >= H) {
        mask[idx] = false;
        return;
    }
    for (int c = 0; c < 3; ++c) {
        features_dc[idx * 3 + c] = image[c * H * W + y * W + x];
    }

    mask[idx] = true;
}

void DELAUNAY_TRIANGULATION::delaunay_triangulation(
    const int N,
    const int H, 
    const int W,
    const float* pcd, 
    const float* image,
    const float* viewmatrix, 
    const float* projmatrix, 
    float* triangles,
    float* features_dc,
    const float min_dist,
    const float max_dist,
    const int grid,
    const float dist_threshold
) 
{
    thrust::device_vector<float> d_point2d(N * 2);
    thrust::device_vector<float> d_grid_record(H * W, 0.0f);
    thrust::device_vector<bool> d_mask(N, false);

    int blocks = (N + 255) / 256;
    project_point<3><<<blocks, 256>>>(
        N, H, W, pcd, viewmatrix, projmatrix,
        thrust::raw_pointer_cast(d_point2d.data()),
        min_dist, max_dist, grid,
        thrust::raw_pointer_cast(d_grid_record.data()),
        thrust::raw_pointer_cast(d_mask.data())
    );
}