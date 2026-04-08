#include "delaunay-triangulation.h"
#include <cooperative_groups.h>
#include <cooperative_groups/reduce.h>
#include <thrust/gather.h>

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

__global__ void project_point(
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
    if (atomicMax(&grid_record[grid_idx], p_view.z) == 0) { // first point
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
    for (int c = 0; c < C; ++c) {
        features_dc[idx * C + c] = image[c * H * W + y * W + x];
    }

    mask[idx] = true;
}

__global__ void fill_triangle_point(
    int total_pts, 
    const int* indices, 
    const float* pcd, 
    float* out_tri
)
{
    auto idx = cg::this_grid().thread_rank();
    if (idx >= total_pts) return;
    
    int pcd_idx = indices[idx];
    out_tri[idx * 3 + 0] = pcd[pcd_idx * 3 + 0];
    out_tri[idx * 3 + 1] = pcd[pcd_idx * 3 + 1];
    out_tri[idx * 3 + 2] = pcd[pcd_idx * 3 + 2];
}

void prepare_gdel_input(
    int N,
    thrust::device_vector<float>& d_point2d,
    thrust::device_vector<bool>& d_mask,
    GDel2DInput& input,
    thrust::device_vector<int>& d_mapping
) 
{
    int num_valid = thrust::count(d_mask.begin(), d_mask.end(), true);
    
    d_mapping.resize(num_valid);
    thrust::copy_if(
        thrust::make_counting_iterator(0), 
        thrust::make_counting_iterator(N), 
        d_mask.begin(),
        d_mapping.begin(),
        thrust::identity<bool>()
    );

    thrust::device_ptr<Point2> p_points = thrust::device_pointer_cast(reinterpret_cast<Point2*>(thrust::raw_pointer_cast(d_point2d.data())));
    
    input.pointVec.resize(num_valid);
    
    auto it_begin = thrust::make_permutation_iterator(p_points, d_mapping.begin());
    auto it_end   = thrust::make_permutation_iterator(p_points, d_mapping.end());

    thrust::copy(it_begin, it_end, input.pointVec.begin());
}

void prepare_gdel_output(
    const GDel2DOutput& output,
    thrust::device_vector<int>& triangles_index
)
{
    size_t num_triangles = output.triVec.size();
    size_t num_integers  = num_triangles * 3;

    triangles_index.resize(num_integers);

    if (num_triangles == 0) return;

    const int* h_ptr = reinterpret_cast<const int*>(output.triVec.data());

    thrust::copy(
        h_ptr, 
        h_ptr + num_integers, 
        triangles_index.begin()
    );
}

void prepare_output(
    const int N,
    thrust::device_vector<int>& d_triangles_index,
    thrust::device_vector<float>& d_features_dc,
    thrust::device_vector<bool>& d_triangle_mask,
    float* triangles,
    float* features_dc,
    const float* pcd
) {
    int num_valid_triangles = thrust::count(d_triangle_mask.begin(), d_triangle_mask.end(), true);
    using Int3 = thrust::tuple<int, int, int>;
    thrust::device_vector<Int3> valid_tri_indices(num_valid_triangles);
    thrust::device_ptr<Int3> tri_ptr = thrust::device_pointer_cast(reinterpret_cast<Int3*>(thrust::raw_pointer_cast(d_triangles_index.data())));
    thrust::copy_if(
        tri_ptr,
        tri_ptr + N,
        d_triangle_mask.begin(),
        valid_tri_indices.begin(),
        thrust::identity<bool>()
    );

    using Float3 = thrust::tuple<float, float, float>;
    thrust::device_vector<Float3> d_features_dc_tmp;
    d_features_dc_tmp.resize(num_valid_triangles);
    thrust::device_ptr<Float3> feat_ptr = thrust::device_pointer_cast(reinterpret_cast<Float3*>(thrust::raw_pointer_cast(d_features_dc.data())));
    thrust::copy_if(
        feat_ptr,
        feat_ptr + N,
        d_triangle_mask.begin(),
        d_features_dc_tmp.begin(),
        thrust::identity<bool>()
    );
    features_dc = reinterpret_cast<float*>(thrust::raw_pointer_cast(d_features_dc_tmp.data()));

    int threads_fill = num_valid_triangles;
    int blocks_fill = (threads_fill + 255) / 256;
    
    fill_triangle_point<<<blocks_fill, 256>>>(
        threads_fill,
        reinterpret_cast<int*>(thrust::raw_pointer_cast(valid_tri_indices.data())),
        pcd,
        triangles
    );
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
    thrust::device_vector<int> d_mapping;
    GDel2DInput triangulation_input;
    GDel2DOutput triangulation_output;

    int blocks = (N + 255) / 256;
    project_point<<<blocks, 256>>>(
        N, H, W, pcd, viewmatrix, projmatrix,
        thrust::raw_pointer_cast(d_point2d.data()),
        min_dist, max_dist, grid,
        thrust::raw_pointer_cast(d_grid_record.data()),
        thrust::raw_pointer_cast(d_mask.data())
    );

    // TODO: remove the host -> device part in delaunay triangulation
    prepare_gdel_input(
        N,
        d_point2d,
        d_mask,
        triangulation_input,
        d_mapping
    );

    GpuDel().compute(triangulation_input, &triangulation_output);

    thrust::device_vector<int> d_triangles_index;
    prepare_gdel_output(triangulation_output, d_triangles_index);

    int N_TRIANGLES = d_triangles_index.size() / 3;
    thrust::device_vector<bool> d_triangle_mask(N_TRIANGLES, false);
    thrust::device_vector<float> d_features_dc(N_TRIANGLES * 3, 0.0f);
    
    blocks = (N_TRIANGLES + 255) / 256;
    
    colorize_triangle<3><<<blocks, 256>>>(
        N_TRIANGLES,
        H, W,
        pcd, 
        thrust::raw_pointer_cast(d_point2d.data()),
        image,
        thrust::raw_pointer_cast(d_triangles_index.data()),
        thrust::raw_pointer_cast(d_features_dc.data()),
        thrust::raw_pointer_cast(d_triangle_mask.data()),
        dist_threshold
    );

    prepare_output(
        N_TRIANGLES,
        d_triangles_index,
        d_features_dc,
        d_triangle_mask,
        triangles,
        features_dc,
        pcd
    );
}