#include "visual_fea_test/StereoFrame.hpp"
#include "visual_fea_test/Converter.h"
#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pangolin/pangolin.h>
#include <opencv2/core/eigen.hpp>


#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "visual_fea_test/Optimizer.hpp"


using namespace std;
using namespace Stereo;
using namespace ORB_SLAM2;
typedef vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Poses;

template <typename T>
void operator>>(const YAML::Node& node,T& i ){
    i = node.as<T>();
};

int main(int argc, char **argv){

    cout << "hello optimizer!" << endl;

    YAML::Node node = YAML::LoadFile("/home/qk/Documents/dj2_MapPoint/Opti.yaml");
    int FrameId,ShowOptiResult,ShowReproj,AddPlaneEdge,ShowPlane,NearPointNum;
    node["FrameId"] >> FrameId;
    node["ShowOptiResult"] >> ShowOptiResult;
    node["ShowReProj"] >> ShowReproj;
    node["AddPlaneEdge"] >> AddPlaneEdge;
    node["ShowPlane"] >> ShowPlane;
    node["NearPointNum"] >> NearPointNum;

    Eigen::Matrix4d Tlc;
    Tlc << -0.0266583,    0.0145156,    0.999539,   -0.00581044,
            -0.999635,   -0.000466258,   -0.0265932,    0.24939,
                0.000427441,   -0.999884,    0.0146246,   -0.16,
                    0,         0,         0,    1.0000;
    
    const double fx = 870.4934000000001;
    const double fy = 870.4934000000001;
    const double cx = 689.5979919433594;
    const double cy = 233.582389831543;
    const double bf = 445.39666;
    Eigen::Matrix<double,5,1> Cam;
    Cam << fx,fy,cx,cy,bf;

    LidarVisualOptimizer Opti(Tlc,Cam);
    Opti.AddPlaneEdge = AddPlaneEdge;
    Opti.NearPointNum = NearPointNum;

    //Step1 . Read Visual Frame and Mappoint
    Opti.ReadTxT("/home/qk/Documents/dj2_MapPoint");
    cout << "ViusalFrame size = " << Opti.VisualFrames.size() << endl;
    
    //Step2 . Read Lidar Map Pcd
    Opti.ReadPcd("/home/qk/Documents/dj2_MapPoint/LidarMap.pcd");
    cout << "Lidar point size = " << Opti.cloud_lidar->points.size() << endl;

    //Step3 . Read G2OFilse from interactive slam 
    // 只根据LidarNearFrame的长度读取所需部分
    Opti.Readg2o("/home/qk/Documents/dj2_MapPoint/graph.g2o");
    // Opti.Readg2o("/home/qk/Desktop/dj2_withPlane_1/graph.g2o");
    cout << "SE3 Edge Num = " << Opti.SE3_Info.size() << endl;
    
    //Step4 . Extract Plane
    Opti.GetPlanePoint();
    Opti.GetPlaneNorm();
    if(ShowPlane == 1)
        Opti.ShowPlaneEx();

    //Step5 . Init Optimizer
    Opti.InitOptimizer();
    Opti.AddVE();
    Opti.Optimize(20);

    if(ShowReproj == 1) 
        ReProj2Pic(Opti,FrameId);

    if(ShowOptiResult == 1)
        Opti.OptiResultShow();

}