README
===

Overall
---
This project is trying to obtain a combination of LIO and Gaussian Splatting + SLAM, I refer to the paper of FastLIVO2, GS-LIVO, GS-SLAM, WildGS-SLAM..., and trying to make a ROS2 version code for them.

Currently, the implement is the LIO part of FastLIVO2, I rewrite it with ROS2, the logic are mostly same as it, just made some change on plane creation function logic (prefix sum and no need to store points if I'm not going to use the VO part of FastLIVO2) and add the LRU cache (as some issue mentioned).

I had tested on MacOS and Linux with docker, and also test the bag on HKU MB, Retail Street dataset, seems it can work now, just the running speed is not quite good, but the memory usage is quite good though.

### Update 2026/03/27: dev branch will no longer support macos and only work on NVidia platform (blackwell).

Install and Test
---

1. build image, start container & build the project in container
```
  git clone https://github.com/peichunhuang-1/GS_LIO.git
  cd GS_LIO
  docker built -t gs_lio .
  bash start.sh
  cd /ros2_ws && source install/setup.bash && colcon build
```
2. attach to it and run bag and publish static transform of imu_link and lidar_link
```
  docker exec -it your-random-container bash
  cd /ros2_ws && source install/setup.bash
  # (MID360)
  ros2 run tf2_ros static_transform_publisher -0.011 -0.02329 0.04412 0 0 0 1 lidar_link imu_link
```
```
  docker exec -it your-random-container bash
  cd /ros2_ws && source install/setup.bash
  ros2 run gs-lio lio_node 
```
```
  docker exec -it your-random-container bash
  cd /ros2_ws && source install/setup.bash
  ros2 bag play your-bag
```

TODO:
---
~~1. running speed: running speed should not be that slow, still finding where is the bottle neck~~

~~2. pure GS part (training and render in c++): my perfect dream GS model is triangle-splatting, I already make a c++ version of triangle-splatting renderer, but trainer are now still writing.~~

3. GS-SLAM part: still understanding GS-SLAM and WildGS-SLAM, I think if we have lidar points, we can save much calculation cost in WildGS-SLAM and GS-SLAM, and hope it can run realtime on edge computing system like GS-LIVO, long way to go...

4. Triangluation for pointcloud: The future work is to conduct triangulation on pointcloud, Currently the plane is use delaunay triangulation in image frame (not like the Immesh in 3d space and projection on voxel plane), I will collect the pcd and project it to keyframe plane, do the delaunay directly on image plane.

5. Sliding window for pcd to train: Like GS-LIVO, we must manage the points dynamically to fulfill a realtime performance, now I have not come up with a good idea.

概覽
---
這個專案是想結合GS-SLAM和LIO，主要是我在用GS-SLAM時感覺他的SLAM的部分不知道是不是有什麼運動上的限制，估測的不是很準，我很希望可以同時做建模跟估測，這樣可以節省大量的儲存空間，bag檔案總是很大的不便於儲存的．
目前主要就是把FastLIVO2的LIO部分做了一些小改動，主要的邏輯不變，我把平面建構的算法稍微修正了一下，因為我不打算使用原本的VO部分，所以不需要儲存raw points，而這樣一來我們可以簡單推導後得到一個prefix sum的公式，去計算平面的不確定性等，理論上應該可以節省儲存空間和計算時間，但計算時間不曉得哪裡有問題並沒有提速．
另外就是我把某些issue提到的LRU cache實作了一下，目前這個版本跑HKU MB大約只會佔用500-600MB左右的記憶體，還算是挺滿意的．目前在MacOS和Linux上測試過，都用docker跑的，測試的資料集只測試過HKU MB和Retail Street．

### 2026/03/27更新：主要是加入了之前提到的triangle-splatting部份，因此目前不再支援Mac平台，針對舊的程式的性能優化上，主要是用welford改了一下prefix sum的部份，希望能提昇數值準確性，先前提到的性能問題，在移除用不到的互斥鎖之後有改進一點，將暫時先不管效能繼續迭代功能的部份。

安裝
---
同英文版跑指令即可．

待辦
---
~~1. 修改掉速度反而更慢的問題：我是覺得應該要比較快或持平才對，之前寫過一版LIVO是基本持平，但因為偷掉了平面計算的遍歷每個點的部分，我倒是有在這個專案試過如果用舊的平面計算方法只會更慢，所以應該是哪裡可以優化寫法而不是算法的問題．~~

~~2. 純粹的高斯搬遷到c++：如題，就是搬運GS到c++上，我的理想模型是想用triangle-splatting，目前完成到可以渲染train完的模型並計算反向傳播了．~~

3. GS-SLAM的部分：還在詳讀代碼跟文章，目前是覺得GS-SLAM裡面的深度那些的可以用lidar點應該問題不大，WildGS-SLAM主要是看上了他的那個不確定性更新，最後是希望可以跟GS-LIVO一樣realtime跑在嵌入式裝置，但應該還有漫漫長路．

4. 三角化點雲：參照Immesh的演算法，但我想直接將一段時間的點雲累積投影到圖像平面上，在圖像平面直接做德勞內三角化。

5. 動態管理三角：要能即時運行在嵌入式裝置上就得動態管理載入訓練的三角面數量，基本上應該會參照GS-LIVO的作法。

參考專案
---
特別感謝以下的開源專案，節省了很多重造車輪的時間，本來是想盡量用submodule之類的方式去引用這些專案，但因為有些是需要稍微改動的，這樣得開一堆fork，管理起來稍微複雜，且基本上這些submodule我之後是不會去改它，待有空再去把一些第三方程式用模組管理。

1. https://github.com/hku-mars/FAST-LIVO2.git
2. https://github.com/HKUST-Aerial-Robotics/GS-LIVO.git
3. https://github.com/trianglesplatting/triangle-splatting.git
4. https://github.com/GradientSpaces/WildGS-SLAM.git
5. https://github.com/yanchi-3dv/diff-gaussian-rasterization-for-gsslam.git
6. https://github.com/graphdeco-inria/gaussian-splatting.git
7. https://github.com/ybubnov/torch_delaunay
