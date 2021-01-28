# readGMappingSLAM

GMapping SLAM算法源代码阅读，加了一些注释，去掉了一些有的没的。

GMapping SLAM源码，参考：

>C++源代码：https://github.com/ros-perception/openslam_gmapping.git

>ROS Package：https://github.com/ros-perception/slam_gmapping.git

相关文章，参考：

- [GMapping SLAM基础、原理与实现（一）：机器人底盘与轮式里程计](https://luolichq.github.io/2019/10/16/gmappingslam-basis-principle-impl-01/)
- [GMapping SLAM基础、原理与实现（二）：激光雷达与数据获取](https://luolichq.github.io/2019/10/17/gmappingslam-basis-principle-impl-02/)
- [GMapping SLAM基础、原理与实现（三）：移动机器人SLAM数学模型](https://luolichq.github.io/2019/10/17/gmappingslam-basis-principle-impl-03/)
- [GMapping SLAM基础、原理与实现（四）：概率学基础知识、贝叶斯滤波器、以及粒子滤波器](https://luolichq.github.io/2019/10/18/gmappingslam-basis-principle-impl-04/)
- [GMapping SLAM基础、原理与实现（五）：里程计运动模型更新采样，与激光雷达似然场观测模型](https://luolichq.github.io/2019/10/19/gmappingslam-basis-principle-impl-05/)


if make compile return link error like this
/usr/bin/ld: 找不到 -lopencv_dep_nppc
/usr/bin/ld: 找不到 -lopencv_dep_nppial
/usr/bin/ld: 找不到 -lopencv_dep_nppicc
/usr/bin/ld: 找不到 -lopencv_dep_nppicom
/usr/bin/ld: 找不到 -lopencv_dep_nppidei
/usr/bin/ld: 找不到 -lopencv_dep_nppif
/usr/bin/ld: 找不到 -lopencv_dep_nppig
/usr/bin/ld: 找不到 -lopencv_dep_nppim
/usr/bin/ld: 找不到 -lopencv_dep_nppist
/usr/bin/ld: 找不到 -lopencv_dep_nppisu
/usr/bin/ld: 找不到 -lopencv_dep_nppitc
/usr/bin/ld: 找不到 -lopencv_dep_npps
/usr/bin/ld: 找不到 -lopencv_dep_cublas
/usr/bin/ld: 找不到 -lopencv_dep_cufft

run bash link_opencv_dep_npp.sh to make soft link from libcudart.so to libopencv_dep_cudart.so ...

make sure that libcublas.so is exist in cuda directory
use `locate libcublas.so` to locate this lib file
this file maybe locate at /usr/lib/x86_64-linux-gnu/libcublas.so, copy to cuda dir