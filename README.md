# LidarStereoRe
This is my undergraduate graduation design project : The Dense 3D Reconstruction of Lidar Stereo Joint.



### 依赖

#### ros

安装对应ros版本即可

#### pangolin

1. 下载源代码

   ```
   git clone https://github.com/stevenlovegrove/Pangolin.git
   ```

2. 安装依赖

   ```
   sudo apt-get install libglew-dev
   sudo apt-get install cmake
   sudo apt-get install libpython2.7-dev
   sudo apt-get install ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev
   sudo apt-get install libdc1394-22-dev libraw1394-dev
   sudo apt-get install libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev
   ```

3. 编译

   ```
   cd Pangolin
   mkdir build
   cd build
   cmake ..
   cmake --build .
   ```

4. linux下安装Pangolin包时提示“No package ‘xbcommon’ found 的方法

   ```
   sudo apt-get install libxkbcommon-dev
   ```

#### g2o

#### Eigen

#### Opencv

