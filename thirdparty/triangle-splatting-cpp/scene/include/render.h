#pragma once
#include "camera.h"
#include "triangle_model.h"
#include "torch/script.h"
#include "diff_triangle_rasterization.h"

torch::autograd::tensor_list render(
    const Camera &camera,
    const TriangleModel &model
);