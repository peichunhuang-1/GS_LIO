include_directories(src)
add_library(gDel2D
    src/gDel2D/GPU/GpuDelaunay.cu
    src/gDel2D/GPU/ThrustWrapper.cu
    src/gDel2D/GPU/KerPredicates.cu
    src/gDel2D/GPU/KerDivision.cu
    src/gDel2D/GPU/SmallCounters.cu
)
set(CMAKE_VERBOSE_MAKEFILE "ON")
target_include_directories(gDel2D PUBLIC src)