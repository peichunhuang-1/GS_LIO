#!/bin/bash
set -e

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia 

source /ros2_ws/install/setup.bash

exec /bin/bash
