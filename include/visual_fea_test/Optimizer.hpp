#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
// #include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include <g2o/core/base_vertex.h>
#include "g2o/core/base_binary_edge.h"
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam3d/edge_se3.h>
#include "visual_fea_test/StereoFrame.hpp"
#include "visual_fea_test/Converter.h"
#include <iostream>
#include <fstream>


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <visual_fea_test/StereoFrame.hpp>
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
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace Stereo;
using namespace ORB_SLAM2;

typedef Eigen::Matrix<double,3,1,Eigen::ColMajor>                               Vector3D;

struct MapPoint_Plane{
    vector<Eigen::Vector4d> Plane_coffs;   
    vector<vector<Stereo::MapPoint>> MapPoints;
    vector<Stereo::MapPoint> MapPoint;
    vector<double> disTH;
    vector<vector<int>> FrameId; 
    vector<int> PlaneId; 
};


class MyEdgeSE3 : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MyEdgeSE3();

    bool read(std::istream& is);

    bool write(std::ostream& os) const;

    void computeError()  {
      const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
      const g2o::VertexSE3Expmap* v2 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);

      g2o::SE3Quat C(_measurement);
      g2o::SE3Quat error_= v2->estimate().inverse()*C*v1->estimate();
      _error = error_.log();
    }

    void linearizeOplus();
};
typedef Eigen::Matrix<double,4,1>  Vector4D;
class VertexPlane : public g2o::BaseVertex<4, Vector4D>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

    virtual bool read(std::istream& is) {};
    virtual bool write(std::ostream& os) const {};

    virtual void setToOriginImpl()
    {
        _estimate << 0,0,0,0;
    }

    virtual void oplusImpl(const double* update)
    {
      Vector4D v(update);
      _estimate += v;
    }

};

class EdgePointPlane : public g2o::BaseBinaryEdge<1,double,g2o::VertexSBAPointXYZ,VertexPlane>
{

    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgePointPlane() {};

    virtual bool read( istream& in) {}
    virtual bool write( ostream& out) const {}

    void computeError()
    {
        const g2o::VertexSBAPointXYZ *v0 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        const VertexPlane *v1 = static_cast<const VertexPlane *>(_vertices[1]);
        Vector3D Point = v0->estimate();
        Vector4D Plane = v1->estimate();
        // cout << "Plane = " << Plane << endl;
        double d  = Plane(0)*Point(0) + Plane(1)*Point(1) +Plane(2)*Point(2) + Plane(3);
        // cout << "error = " << d << endl;
        _error(0,0) =  d - _measurement;
    }

    virtual void linearizeOplus();

};


class My3Edge : public g2o::BaseMultiEdge<3 ,Vector3D > 
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    My3Edge() {};
    virtual bool read( istream& in) {}
    virtual bool write( ostream& out) const {}
    void computeError()
    {
        // cout << "Error  " << endl;
        const g2o::VertexSE3Expmap *v0 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[2]);
        const g2o::VertexSBAPointXYZ *v2 = static_cast<const g2o::VertexSBAPointXYZ *>(_vertices[0]);
        Vector3D obs(_measurement);
        Vector3D Pc;
        Vector3D Pl;
        Pl = v1->estimate().map(v2->estimate());
        Pc = v0->estimate().map(Pl);
        _error = obs - cam_project(Pc,bf);
    }

    Vector3D cam_project(const Vector3D &trans_xyz, const float &bf) const;
    virtual void linearizeOplus();
    double fx, fy, cx, cy, bf;
};

struct EdgeSE3Info{
  int Id1;
  int Id2;
  g2o::SE3Quat dT;
};

struct EdgeSE3PlaneInfo{
  int Id1;
  int Id2;
  Eigen::Vector4d Plane;
};


class LidarVisualOptimizer{
public:
    LidarVisualOptimizer() {};
    ~LidarVisualOptimizer() {};

    LidarVisualOptimizer(Eigen::Matrix4d T,Eigen::Matrix<double,5,1> Cam);
    void ReadTxT(string Path);
    void ReadPcd(string Path);
    void Readg2o(string Path);
    void SetTlc(Eigen::Matrix4d T);
    void GetPlanePoint();
    void ShowPlaneEx();
    void InitOptimizer();
    void AddVE();
    void Optimize(int Num);
    void OptiResultShow();
    g2o::SE3Quat T2SE3Quat(Eigen::Matrix4d T);
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Frame2Isom(vector<Stereo::kFrame> Frames);
    void GetPlaneNorm();

    // Visual Frame and Mappoint
    vector<Stereo::kFrame> VisualFrames;
    vector<Stereo::MapPoint> VisualMapPoints;
    vector<Stereo::kFrame> VisualFrames_raw;
    vector<Stereo::MapPoint> VisualMapPoints_raw;

    // Lidar Frame 
    vector<Stereo::kFrame> LidarFrames;

    // Lidar Near Frame Id
    vector<int> LidarNearFrame;

    // Plane 
    MapPoint_Plane PlaneAndPoint;
    MapPoint_Plane PlaneAndPoint_Norm;

    // Lidar Planes
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds_plane;

    // Lidar Pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visual;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_norm_;

    // Edge SE3
    vector<EdgeSE3Info> SE3_Info;
    vector<EdgeSE3PlaneInfo> SE3_Plane_Info;
    // Optimizer
    g2o::SparseOptimizer optimizer;
    float thHuber3D;
    int VertexNum = 0;
    int IdVisual;
    int IdPoint;
    int IdPlane;
    int IdLidar;
    int IdNormPlane;

    // Camera
    double fx ;
    double fy ;
    double cx ;
    double cy ;
    double bf ;

    int AddPlaneEdge;
    int NearPointNum;

    Eigen::Matrix4d Tlc;
    Eigen::Matrix4d Tlc_o;
    Eigen::Matrix4d Tcl;



};


class ReProj{
public:
  ReProj() {};
  ~ReProj() {};
  

  string ImPath;
  double fx;
  double fy;
  double cx;
  double cy;
  double bf;

  Eigen::Matrix4d Tlc;
  Eigen::Matrix4d Twl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_visual;

};


void ReProj2Pic(LidarVisualOptimizer &Opti,int FrameId);


#endif 