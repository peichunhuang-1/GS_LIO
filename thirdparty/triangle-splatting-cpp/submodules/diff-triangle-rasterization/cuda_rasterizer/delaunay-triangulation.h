#ifndef CUDA_RASTERIZER_DELAUNAY_TRIANGULATION_H_INCLUDED
#define CUDA_RASTERIZER_DELAUNAY_TRIANGULATION_H_INCLUDED

#include "auxiliary.h"
#include "gDel2D/GpuDelaunay.h"
#include <cooperative_groups.h>
#include <cooperative_groups/reduce.h>
#include <thrust/gather.h>

namespace DELAUNAY_TRIANGULATION
{

void delaunay_triangulation(
    const int N,
    const int H, 
    const int W,
    const float* pcd, 
    const float* image,
    const float* viewmatrix, 
    const float* projmatrix, 
    float* triangles,
    float* features_dc,
    int& num_triangles,
    const float min_dist, // depth space threshold
    const float max_dist, // depth space threshold
    const int grid, // pixel space threshold
    const float dist_threshold // 3d space distance threshold
);

}

#endif