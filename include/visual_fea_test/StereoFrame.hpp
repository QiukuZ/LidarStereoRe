#ifndef STEREOFRAME_H
#define STEREOFRAME_H

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <string>
#include "visual_fea_test/ORBextractor.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "visual_fea_test/OctTree.hpp"
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZ PointT3;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT3> PointCloud3;
using namespace std;
namespace Stereo{

struct PointInfo{
    double dis;
    cv::Point2i p;
    Eigen::Vector3d Pc;
    cv::Mat Desc;
};

struct Observation{
    int FrameID;
    int FeaIndex;
};
struct MapPoint{
    std::vector<Observation> Observations; 
    Eigen::Vector3d WorldPos;
    cv::Vec3b BGR;
    cv::Mat desc;
    int newest_frame;
    int id;
    bool Is_good;
};

struct FeaPoint{
public:
    cv::Point2i p;
    double dis;
    cv::Mat desc;
    int Frame_id; //对应frame id
    int id;
};

struct kFrame{
    std::vector<FeaPoint> FeaPoints;
    int id;
    Eigen::Matrix4d pose;
    ros::Time timestamp;
    double time;
    double info;
    bool Near;
};

class StereoCamera
{
public:
    StereoCamera();
    ~StereoCamera();

    void GetParm(const std::string Path);
    void ReMap();
    void ComputeDispAndReProj(cv::Mat &disp);
    void InitSGBM();
    void Tracking();
    void Tracking2();
    void GetCloudPoint();
    void ComputeMatch_Ransac(std::vector<cv::KeyPoint> Last_KP,cv::Mat Last_Desc,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame frame,kFrame frame_last);
    void ComputeMatch(std::vector<cv::KeyPoint> Last_KP,cv::Mat Last_Desc,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame &frame,kFrame &frame_last);
    void ComputeMatch(kFrame LastF,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame &frame,std::vector<cv::KeyPoint> &KP_new,cv::Mat &Desc_new);
    void GetPCL(std::vector<MapPoint> Mps,std::string path);
    void WriteMapPoints(std::string Mppath);
    void WritekFrames(std::string kFpath);
    double ErrorReProj(cv::Point2f p,Eigen::Matrix4d dR,Eigen::Vector3d P);
    // 标定参数
    cv::Mat Left_K; // 左目相机内参矩阵
    cv::Mat Left_D; // 左目相机畸变矩阵
    cv::Mat Right_K; // 右目相机内参矩阵
    cv::Mat Right_D; // 右目相机畸变矩阵
    cv::Mat Stereo_r; // 相机之间旋转向量
    cv::Mat Stereo_T; // 相机之间平移向量
    double bf;
    cv::Size ImageSize;

    // 计算参数
    double RePerTH;
    cv::Mat Left_R;
    cv::Mat Left_P;
    cv::Mat Right_R;
    cv::Mat Right_P;
    cv::Mat Q;
    cv::Mat Q_inv;
    cv::Rect validROIL;
    cv::Rect validROIR;

    Eigen::Matrix4d Tbc;
    Eigen::Matrix4d Tbc_inv;
    Eigen::Matrix4d Pose;
    Eigen::Matrix4d Pose_last;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Eigen::Matrix3d Left_Kinv;
    cv::Mat Left_Im;
    cv::Mat Left_reIm;

    //last
    cv::Mat Left_lastreIm;
    cv::Mat Right_lastreIm;
    cv::Mat Left_lastDesc;
    std::vector<cv::KeyPoint> vLeft_lastKp;
    // cv::Mat Left_lastDesc;

    cv::Mat Right_Im;
    cv::Mat Right_reIm;
    cv::Mat Last_Disp;
    ros::Time Left_Time;
    ros::Time Right_Time;

    std::vector<kFrame> Frames;
    int NumofFrame;
    int NumofFea; //用来计算所有的特征点id
    bool FirstMatch;
    bool UseRansac;
    std::vector<MapPoint> MapPoints;
    std::vector<MapPoint> MapPoints_Good;

    //for test 
    cv::Mat Left_lastlastreIm;
    cv::Mat Left_lastlastDesc;
    std::vector<cv::KeyPoint> vLeft_lastlastKp;
    std::vector<cv::DMatch> last_match;
    std::vector<int> LidarNearFrame;
    bool NearLidar;

    Eigen::Matrix4d PnPPose;
    
};

class Frame
{
public:
    Frame();
    ~Frame();
    cv::Mat Left_Im;
    cv::Mat Right_Im;
    int N ;  
    int height; 
    int width;
    ros::Time LeftTime;
    ros::Time RightTime;

};

int ComputeDist(const cv::Mat &a,const cv::Mat &b);


}


#endif 