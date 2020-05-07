# LidarStereoRe
This is my undergraduate graduation design project : The Dense 3D Reconstruction of Lidar Stereo Joint.



### 依赖

#### ros

#### pangolin

#### Eigen

#### Opencv

#### yaml-cpp



### 1.StereoFeature

```
roslaunch LidarStereoRe StereoFeature.launch
```

- 输入
  - rosbag - 双目图像
  - yaml - 标定参数
  - pose - 激光里程计获得的pose

### 2.Optimizer

