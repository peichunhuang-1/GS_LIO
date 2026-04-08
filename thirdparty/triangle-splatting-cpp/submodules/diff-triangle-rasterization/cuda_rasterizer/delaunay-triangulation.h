#ifndef CUDA_RASTERIZER_DELAUNAY_TRIANGULATION_H_INCLUDED
#define CUDA_RASTERIZER_DELAUNAY_TRIANGULATION_H_INCLUDED

#include "gDel2D/GpuDelaunay.h"
#include "auxiliary.h"

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
    const float min_dist,
    const float max_dist,
    const int grid,
    const float dist_threshold
);

}

#endif