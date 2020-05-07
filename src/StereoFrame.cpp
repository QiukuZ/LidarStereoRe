#include "visual_fea_test/StereoFrame.hpp"
#include <opencv2/core/eigen.hpp>
#include <time.h>
#include "visual_fea_test/ORBextractor.h"
#include "algorithm"
typedef unsigned long size_t;
using namespace std;
namespace Stereo{

void KeyPointsToPoints(const std::vector<cv::KeyPoint>& kps, std::vector<cv::Point2f>& ps)
 {
    ps.clear();
    for (unsigned int i = 0; i<kps.size(); i++) 
        ps.push_back(kps[i].pt);
}

StereoCamera::StereoCamera(void)
{
    NumofFea = 0;
    NumofFrame = 0;
    FirstMatch = true;
}

StereoCamera::~StereoCamera(void){}

/**
 * 读取相机标定参数
 * 计算得到 R P Q
 */
void StereoCamera::GetParm(const std::string Path)
{
    cv::FileStorage fs(Path,cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        ROS_ERROR("Failed to open yaml file!");
        return;
    }

    fs["LEFT.K"] >> Left_K;
    fs["LEFT.D"] >> Left_D;
    fs["RIGHT.K"] >> Right_K;
    fs["RIGHT.D"] >> Right_D;
    fs["R"] >> Stereo_r;
    fs["T"] >> Stereo_T;
    fs["Camera.width"] >> ImageSize.width;
    fs["Camera.height"] >> ImageSize.height;
    fs["Camera.bf"] >> bf;

    cv::Mat Stereo_R;
    cv::Rodrigues(Stereo_r,Stereo_R);
    Stereo_T = Stereo_T.reshape(1,3);
    Left_D = Left_D.reshape(1,5);
    Right_D = Right_D.reshape(1,5);

    cv::stereoRectify(Left_K,Left_D,Right_K,Right_D,ImageSize,Stereo_R,Stereo_T,Left_R,Right_R,Left_P,Right_P,Q,cv::CALIB_ZERO_DISPARITY,-1,ImageSize,&validROIL,&validROIR);
    cout << "Q " << endl << Q << endl;
    Q_inv = Q.inv();
    // cout << "Q inv " << endl << Q_inv <<endl;
    Eigen::Matrix3d K;
    
    cv::cv2eigen(Left_K,K);
    Left_Kinv = K.inverse();

    Tbc << -0.0266583,    0.0145156,    0.999539,   -0.00581044,
   -0.999635,   -0.000466258,   -0.0265932,    0.24939,
    0.000427441,   -0.999884,    0.0146246,   -0.16,
         0,         0,         0,    1.0000;
    // Tbc << 0,0,1,0.06,
    //     -1,0,0, 0.2569,
    //     0,-1,0, -0.15,
    //     0, 0,0, 1.0000;

    Tbc_inv = Tbc.inverse();
    ROS_INFO("Read Camera Parm success!");

}

/**
 * 输入：左右目相机图像 输出：校正后双目图像
 * 对图像进行双目校正
 */

void StereoCamera::ReMap()
{

    cv::Mat rmap[2][2];
    cv::initUndistortRectifyMap(Left_K,Left_D,Left_R,Left_P,ImageSize,CV_16SC2,rmap[0][0],rmap[0][1]);
    cv::initUndistortRectifyMap(Right_K,Right_D,Right_R,Right_P,ImageSize,CV_16SC2,rmap[1][0],rmap[1][1]);
    // cout << " Right P = " << endl << Right_P << endl;
    // cv::remap(Left_Im,Left_reIm,rmap[0][0],rmap[0][1],cv::INTER_AREA);
    // cv::remap(Right_Im,Right_reIm,rmap[1][0],rmap[1][1],cv::INTER_AREA);
    cv::remap(Left_Im,Left_reIm,rmap[0][0],rmap[0][1],cv::INTER_LINEAR);
    cv::remap(Right_Im,Right_reIm,rmap[1][0],rmap[1][1],cv::INTER_LINEAR);
}

/**
 *  初始化sgbm
 *  
 */
void StereoCamera::InitSGBM()
{
    // cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    // int numberOfDisparities = ((ImageSize.width / 8) + 15) & -16;
    // sgbm->setPreFilterCap(63);
    // int SADWindowSize = 9;
    // int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    // sgbm->setBlockSize(sgbmWinSize);
    // int cn = 3;
    // sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    // sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    // sgbm->setUniquenessRatio(10);
    // sgbm->setMinDisparity(0);
    // sgbm->setNumDisparities(numberOfDisparities);
    // sgbm->setSpeckleWindowSize(100);
    // sgbm->setSpeckleRange(32);
    // sgbm->setDisp12MaxDiff(1);
    // sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
}


/**
 *  输入：左右目矫正后图像 输出 视差图
 *  计算视差图
 */

void StereoCamera::ComputeDispAndReProj(cv::Mat &disp)
{
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,16,3);
    int numberOfDisparities = ((Left_reIm.size().width / 8) + 15) & -16;
    sgbm->setPreFilterCap(63);
    int SADWindowSize = 9;
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);
    int cn = Left_reIm.channels();
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setUniquenessRatio(10);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    sgbm->compute(Left_reIm,Right_reIm,disp);
}



Frame::Frame(void){
    ROS_INFO("Create a Frame!");
    N = 700;
}

Frame::~Frame(void){}


void StereoCamera::GetCloudPoint()
{
    uint64_t time_= Left_Time.toNSec();
    PointCloud::Ptr cloud(new PointCloud);
    
    ReMap();
    cv::Mat disp;
    ComputeDispAndReProj(disp);
    int numberOfDisparities = ((ImageSize.width / 8) + 15) & -16;
    // cv::Mat disp8;
    // disp.convertTo(disp8,CV_8U,255/(numberOfDisparities*16.));
    // cv::imshow("im",disp8);
    // cv::waitKey(100);
    for (int iu = 0; iu < ImageSize.width; iu++)
    {
        for (int iv = 0; iv < ImageSize.height; iv++)
        {
            cv::Point p;
            p.x = iu;
            p.y = iv;
            unsigned short dis = disp.at<unsigned short>(p);
            if (dis > 500 || dis <40)
                continue;
            
            cv::Vec3b temp_color = Left_reIm.at<cv::Vec3b>(p);
        
            PointT pc;
            pc.z = bf/(float)disp.at<unsigned short>(p);
            std::cout << "z = " << pc.z << std::endl;
            pc.x = pc.z*((float)p.x-(float)Left_K.at<double>(0,2))/(float)Left_K.at<double>(0,0);
            pc.y = pc.z*((float)p.y-(float)Left_K.at<double>(1,2))/(float)Left_K.at<double>(1,1);
            pc.r = temp_color[2];
            pc.g = temp_color[1];
            pc.b = temp_color[0];
            cloud->points.push_back(pc);
            // std::cout << "x :" <<pc.x << "  y:" << pc.y << "  z:" << pc.z << std::endl;
        }
        
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    std::string N_ = std::to_string(time_);
    std::string path_pcd = "/home/qk/Desktop/stereo_pcl_test/" + N_ + ".pcd" ;
    pcl::io::savePCDFile(path_pcd, *cloud);
}

void StereoCamera::GetPCL(std::vector<MapPoint> Mps,std::string path)
{
    PointCloud::Ptr cloud(new PointCloud);
    for (int im = 0; im < Mps.size(); im++)
    {
        PointT pc;
        pc.x = Mps[im].WorldPos(0);
        pc.y = Mps[im].WorldPos(1);
        pc.z = Mps[im].WorldPos(2);
        pc.b = Mps[im].BGR[0];
        pc.g = Mps[im].BGR[1];
        pc.r = Mps[im].BGR[2];
        cloud->points.push_back(pc);
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    pcl::io::savePCDFile(path, *cloud);
}

void StereoCamera::Tracking()
{
    ReMap();
    cv::Mat disp;
    ImageSize = Left_Im.size();
    // cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(1200);
    std::vector<cv::KeyPoint> vLeft_Kp;
    cv::Mat Left_Desc;

    int height = Left_Im.size().height;
    int width = Left_Im.size().width;
    
    // SIFT特征点和关键帧
    detector->detect(Left_reIm,vLeft_Kp);
    detector->compute(Left_reIm,vLeft_Kp,Left_Desc);

    kFrame ThisFrame;
    std::vector<cv::KeyPoint> KP_new;
    cv::Mat Desc_new;

    

    if (NumofFrame == 0)
    {
        //只将图像传递给上一帧 并将关键帧和描述子保留
        Left_lastreIm = Left_reIm.clone();
        vLeft_lastKp.clear();
        for (int i = 0; i < vLeft_Kp.size(); i++)
            vLeft_lastKp.push_back(vLeft_Kp[i]);
        Left_lastDesc = Left_Desc.clone();
        ThisFrame.pose = Pose;
        ThisFrame.id = NumofFrame;
        Frames.push_back(ThisFrame);
        NumofFrame ++;
        return;
    }
    else if (NumofFrame == 1)
    {
        // 这一帧由于第一次进行双目匹配，只进行新特征的提取
        kFrame frame_new,frame_last;
        // 得到 当前 和 上一frame 的 特征点
        ComputeMatch_Ransac(vLeft_lastKp,Left_lastDesc,vLeft_Kp,Left_Desc,frame_new,frame_last);
        ComputeMatch(vLeft_lastKp,Left_lastDesc,vLeft_Kp,Left_Desc,frame_new,frame_last);
        std::cout << "New Match = " << frame_new.FeaPoints.size() << std::endl;
        // 此时上一帧也是空的
        for (int fi = 0; fi < frame_new.FeaPoints.size(); fi++)
        {
            
            // 1.  New MapPoint
            MapPoint mp;
            mp.id = frame_new.FeaPoints[fi].id;
            double z = bf/frame_new.FeaPoints[fi].dis;
            mp.BGR = Left_reIm.at<cv::Vec3b>(frame_new.FeaPoints[fi].p);
            
            Eigen::Vector4d Pw;
            Eigen::Vector3d puv((double)frame_new.FeaPoints[fi].p.x,(double)frame_new.FeaPoints[fi].p.y,1.0);
    
            Eigen::Vector3d Pc;
            Pc = z*Left_Kinv*puv;
            // cv::Mat imleft;
            // imleft = Left_reIm.clone();
            // cv::circle(imleft,frame_new.FeaPoints[fi].p,3,CV_RGB(0,0,0),2);
            // cv::Mat imlast;
            // imlast = Left_lastreIm;
            // cv::circle(imleft,frame_last.FeaPoints[fi].p,3,CV_RGB(0,0,0),2);
            // cv::hconcat(imleft,imlast,imleft);
            // cv::imshow("imleft",imleft);
            // cv::waitKey();

            // std::cout <<"****"<<std::endl;
            // std::cout << "Pc" << std::endl << Pc <<std::endl;
            // std::cout << "dR " << std::endl<< Frames[NumofFrame-1].pose.inverse()*Pose<< std::endl;
            // std::cout << "u = "<< frame_last.FeaPoints[fi].p.x << "v = " << frame_last.FeaPoints[fi].p.y << std::endl;

            Pw(0) = Pc(0);
            Pw(1) = Pc(1);
            Pw(2) = Pc(2);
            Pw(3) = 1;
            
            Pw = Pose*Tbc*Pw;

            mp.BGR = Left_reIm.at<cv::Vec3b>(frame_new.FeaPoints[fi].p);
            mp.WorldPos(0) = Pw(0);
            mp.WorldPos(1) = Pw(1);
            mp.WorldPos(2) = Pw(2);

            //  在本frame被观测
            Observation ob;
            ob.FeaIndex = NumofFrame;
            ob.FrameID = fi;
            mp.Observations.push_back(ob);
            MapPoints.push_back(mp);
    
            //在本帧也要增加观测
            ThisFrame.FeaPoints.push_back(frame_new.FeaPoints[fi]);
            // 在上一帧 新增观测
            Frames[0].FeaPoints.push_back(frame_last.FeaPoints[fi]);
            Observation ob1;
            ob1.FrameID = 0;
            ob1.FeaIndex = fi;
            MapPoints[mp.id].Observations.push_back(ob1);
            // ROS_INFO(" = 1 ") ;
        }
        Left_lastreIm = Left_reIm.clone();
        vLeft_lastKp.clear();
        for (int i = 0; i < vLeft_Kp.size(); i++)
            vLeft_lastKp.push_back(vLeft_Kp[i]);
        Left_lastDesc = Left_Desc.clone();
    }
    else{
        // 先做旧匹配
        ComputeMatch(Frames[NumofFrame-1],vLeft_Kp,Left_Desc,ThisFrame,KP_new,Desc_new);
        std::cout << "Old Match Observe = " << ThisFrame.FeaPoints.size() << std::endl;
        for (int fi = 0; fi < ThisFrame.FeaPoints.size(); fi++)
        {
            // 旧匹配只需要压入观测特征点 已在ThisFrame中
            int mp_id = ThisFrame.FeaPoints[fi].id;
            Observation ob;
            ob.FrameID = NumofFrame;
            ob.FeaIndex = fi;
            MapPoints[mp_id].Observations.push_back(ob);
            // std::cout << "Old Map id " << mp_id << " Obs by Frame:" << NumofFrame << std::endl; 
        }
        
        //注意，则这时候本帧索引非0 开始
        int index = ThisFrame.FeaPoints.size();
        int index_last = Frames[NumofFrame-1].FeaPoints.size();
        //这时候使用新-旧来提取新特征
        kFrame frame_new,frame_last;
        ComputeMatch_Ransac(vLeft_lastKp,Left_lastDesc,KP_new,Desc_new,frame_new,frame_last);
        ComputeMatch(vLeft_lastKp,Left_lastDesc,KP_new,Desc_new,frame_new,frame_last);
        
        std::cout << "New Match  = " << frame_new.FeaPoints.size() << std::endl;
        for (int fi = 0; fi < frame_new.FeaPoints.size(); fi++)
        {
            // 1.New point
            MapPoint mp;
            mp.id = frame_new.FeaPoints[fi].id;
            double z = bf/frame_new.FeaPoints[fi].dis;
    
            Eigen::Vector4d Pw;
            Eigen::Vector3d puv((double)frame_new.FeaPoints[fi].p.x,(double)frame_new.FeaPoints[fi].p.y,1.0);
    
            Eigen::Vector3d Pc;
            Pc = z*Left_Kinv*puv;
            // std::cout << "Pc " << std::endl << Pc << std::endl;
            Pw(0) = Pc(0);
            Pw(1) = Pc(1);
            Pw(2) = Pc(2);
            Pw(3) = 1;
            
            Pw =Pose*Tbc*Pw;
            // Rbc = Rbc.inverse();
            // Eigen::Matrix3d R_inv = R.inverse();
            // Pw = z*R_inv*Left_Kinv*puv - R_inv*t;
            mp.BGR = Left_reIm.at<cv::Vec3b>(frame_new.FeaPoints[fi].p);
            mp.WorldPos(0) = Pw(0);
            mp.WorldPos(1) = Pw(1);
            mp.WorldPos(2) = Pw(2);

             //在本frame被观测
            Observation ob;
            ob.FrameID = NumofFrame;
            ob.FeaIndex = fi+index;
            mp.Observations.push_back(ob);
            // std::cout << "New Map id " << mp.id << " Obs by Frame:" << NumofFrame << std::endl; 
            MapPoints.push_back(mp);
            ThisFrame.FeaPoints.push_back(frame_new.FeaPoints[fi]);

            //上一帧的观测
            Frames[NumofFrame-1].FeaPoints.push_back(frame_last.FeaPoints[fi]);
            Observation ob1;
            ob1.FrameID = NumofFrame -1;
            ob1.FeaIndex = fi+index_last;
            MapPoints[mp.id].Observations.push_back(ob1);
        }
        Left_lastreIm = Left_reIm.clone();
        vLeft_lastKp.clear();
        for (int i = 0; i < KP_new.size(); i++)
            vLeft_lastKp.push_back(KP_new[i]);
        Left_lastDesc = Desc_new.clone();
        
    }
    std::cout << "*************************************" <<std::endl;
    std::cout << " Frame  " <<  NumofFrame << " done!" <<std::endl;

    ThisFrame.pose = Pose;
    ThisFrame.id = NumofFrame;
    ThisFrame.timestamp = Left_Time;
    Frames.push_back(ThisFrame);
    NumofFrame ++;


}

int ComputeDist(const cv::Mat &a,const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();
    int dist = 0;
    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }
    return dist;
}

//该函数用于新特征匹配
void StereoCamera::ComputeMatch(std::vector<cv::KeyPoint> Last_KP,cv::Mat Last_Desc,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame &frame,kFrame &frame_last)
{
    const size_t last_fnum = Last_KP.size();
    const size_t now_fnum = KP.size();
    std::vector<cv::DMatch> matches_bf;//用于存储暴力检测
    
    int width = ImageSize.width;
    int height = ImageSize.height;
    for (size_t ii = 0; ii < last_fnum; ii++)
    {
        int bestdist = 128;
        int bestidx = 0;
        cv::DMatch m;
        const cv::Mat &dL = Last_Desc.row(ii);
        for (size_t jj = 0; jj < now_fnum; jj++)
        {
            const cv::Mat &dN = Desc.row(jj);
            int dist = ComputeDist(dL,dN);
            if (dist < bestdist)
            {
                // std::cout << "dist = " <<dist << std::endl;
                bestdist = dist;
                bestidx = jj;
            }
        }
        m.distance = bestdist;
        m.queryIdx = ii;
        m.trainIdx = bestidx;
        matches_bf.push_back(m);
    }

    //通过一范数和视察来筛选点
    // std::vector<cv::DMatch> good_match;
    for (size_t i = 0; i < matches_bf.size(); i++)
    {
        //对于好的特征匹配，计算视差
        //左目划窗设置为11
        int w = 5;
        const int &uL = KP[matches_bf[i].trainIdx].pt.x;
        const int &vL = KP[matches_bf[i].trainIdx].pt.y;
        if (uL-w-1 <0 || uL+w+1 > width)
            continue;//过界不要
        else if (vL-w-1<0 || vL+w+1 >height)
            continue;
        
        cv::Mat IL = Left_reIm.rowRange(vL-w,vL+w+1).colRange(uL-w,uL+w+1);
        IL.convertTo(IL,CV_32F);

        int bestDist = __INT_MAX__;
        int bestincR = 0;
        
        for (int incR = w+1; incR < width-w-2; incR++)
        {
            // ROS_INFO("u : %d,%d",incR-w,incR+w+1);
            cv::Mat IR = Right_reIm.rowRange(vL-w,vL+w+1).colRange(incR-w,incR+w+1);
            IR.convertTo(IR,CV_32F);
            float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
            if (dist < bestDist)
            {
                bestDist = dist;
                bestincR = incR;
            }
        }
        int d = uL-bestincR;
        if (d > 20 && d<400)
        {   // good_match.push_back(matches_bf[i]);
            
            //计算投影到上一帧的重投影误差
            Eigen::Vector3d puv((double)uL,(double)vL,1.0);
            Eigen::Vector3d Pc = ((double)bf/(double)d)*Left_Kinv*puv;
            Eigen::Matrix4d dR;
            dR = Frames[NumofFrame-1].pose.inverse()*Pose;
            dR = Tbc_inv*dR*Tbc;
            double Error = ErrorReProj(Last_KP[matches_bf[i].queryIdx].pt,dR,Pc);

            if(Error < RePerTH)
            {

            //完成匹配，将id u v d 存入 fea
                FeaPoint fp;
                fp.p.x = uL;
                fp.p.y = vL;
                fp.id = NumofFea++;
                fp.desc = Desc.row(matches_bf[i].trainIdx);
                fp.dis = (double)d;
                frame.FeaPoints.push_back(fp);

                //同时也要获得上一帧的匹配情况
                FeaPoint fp_last;
                fp_last.p.x = Last_KP[matches_bf[i].queryIdx].pt.x;
                fp_last.p.y = Last_KP[matches_bf[i].queryIdx].pt.y;
                fp_last.id = fp.id;
                fp_last.dis = (double)d;
                frame_last.FeaPoints.push_back(fp_last);
            }
        }
    }
}

void StereoCamera::ComputeMatch(kFrame LastF,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame &frame,std::vector<cv::KeyPoint> &KP_new,cv::Mat &Desc_new)
{
    const size_t last_fnum = LastF.FeaPoints.size();
    const size_t now_fnum = KP.size();
    std::vector<cv::DMatch> matches_bf;//用于存储暴力检测
    std::vector<int> used_index;//用来记录当前特征点中被使用的点
    int width = ImageSize.width;
    int height = ImageSize.height;

    for (size_t ii = 0; ii < last_fnum; ii++)
    {
        int bestdist = 128;
        int bestidx = 0;
        cv::DMatch m;
        const cv::Mat &dL = LastF.FeaPoints[ii].desc;
        for (size_t jj = 0; jj < now_fnum; jj++)
        {
            const cv::Mat &dN = Desc.row(jj);
            int dist = ComputeDist(dL,dN);
            if (dist < bestdist)
            {
                // std::cout << "dist = " <<dist << std::endl;
                bestdist = dist;
                bestidx = jj;
            }
        }
        m.distance = bestdist;
        m.queryIdx = ii;
        m.trainIdx = bestidx;
        matches_bf.push_back(m);
    }

    //通过一范数和视察来筛选点
    // std::vector<cv::DMatch> good_match;
    for (size_t i = 0; i < matches_bf.size(); i++)
    {
        
        //对于好的特征匹配，计算视差
        //左目划窗设置为11
        int w = 5;
        const int &uL = KP[matches_bf[i].trainIdx].pt.x;
        const int &vL = KP[matches_bf[i].trainIdx].pt.y;
        if (uL-w-1 <0 || uL+w+1 > width)
            continue;//过界不要
        else if (vL-w-1<0 || vL+w+1 >height)
            continue;
        
        cv::Mat IL = Left_reIm.rowRange(vL-w,vL+w+1).colRange(uL-w,uL+w+1);
        IL.convertTo(IL,CV_32F);

        int bestDist = __INT_MAX__;
        int bestincR = 0;
        
        for (int incR = w+1; incR < width-w-2; incR++)
        {
            // ROS_INFO("u : %d,%d",incR-w,incR+w+1);
            cv::Mat IR = Right_reIm.rowRange(vL-w,vL+w+1).colRange(incR-w,incR+w+1);
            IR.convertTo(IR,CV_32F);
            float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
            if (dist < bestDist)
            {
                bestDist = dist;
                bestincR = incR;
            }
        }
        int d = uL-bestincR;
        if (d>20 && d<400)
        {
            // good_match.push_back(matches_bf[i]);
            //完成匹配，将id u v d 存入 fea
            //计算投影到上一帧的重投影误差
            Eigen::Vector3d puv((double)uL,(double)vL,1.0);
            Eigen::Vector3d Pc = ((double)bf/(double)d)*Left_Kinv*puv;

            Eigen::Matrix4d dR;
            dR = Frames[NumofFrame-1].pose.inverse()*Pose;
            dR = Tbc_inv*dR*Tbc;
            cv::Point2d p;
            p.x = (double)LastF.FeaPoints[matches_bf[i].queryIdx].p.x;
            p.y = (double)LastF.FeaPoints[matches_bf[i].queryIdx].p.y;
            double Error = ErrorReProj(p,dR,Pc);
            if(Error < RePerTH)
            {
                FeaPoint fp;
                fp.p.x = uL;
                fp.p.y = vL;
                fp.Frame_id = NumofFrame;
                fp.desc = Desc.row(matches_bf[i].trainIdx);
                fp.dis = (double)d;
                //被重复观察到
                fp.id = LastF.FeaPoints[matches_bf[i].queryIdx].id;
                frame.FeaPoints.push_back(fp);
                used_index.push_back(matches_bf[i].trainIdx);
            }
        }
    }
    //在输出的关键点和描述子中删除用过的点
    for (int ki = 0; ki < KP.size(); ki++)
    {
        bool isused = false;
        for (int ui = 0; ui < used_index.size(); ui++)
        {
            if (ki == used_index[ui])
                isused = true;
        }
        if (!isused)
        {
            KP_new.push_back(KP[ki]);
            Desc_new.push_back(Desc.row(ki));
        }
    }
}

double StereoCamera::ErrorReProj(cv::Point2f p,Eigen::Matrix4d dR,Eigen::Vector3d P)
{
    Eigen::Vector4d P_;
    P_(0) = P(0);
    P_(1) = P(1);
    P_(2) = P(2);
    P_(3) = P(3);
    Eigen::Vector4d P_r = dR*P_;

    Eigen::Vector3d Pr;
    Pr(0) = P_r(0);
    Pr(1) = P_r(1);
    Pr(2) = P_r(2);
    double u = Left_K.at<double>(0,0)*Pr(0)/Pr(2) + Left_K.at<double>(0,2);
    double v = Left_K.at<double>(1,1)*Pr(1)/Pr(2) + Left_K.at<double>(1,2);
    double e = (u - (double)p.x)*(u - (double)p.x) + (v - (double)p.y)*(v - (double)p.y);
    return e;
}

void StereoCamera::WriteMapPoints(std::string Mppath)
{
    cout << " Begin to write txt ! ..." << endl;
    std::ofstream ofs(Mppath + "/MapPoints.txt");
    if (!ofs)
    {
        cout << " open fail Mappoints " << endl;
        return;
    }
    ofs.clear();
    for (int i = 0; i < MapPoints.size(); i++)
    {
        // 1. Id 
        int obnum= MapPoints[i].Observations.size();
        if(obnum >=2)
        {
            ofs << "Id ";
            ofs <<  MapPoints[i].id << "\n";
            ofs << "ObsNum " ;
            ofs <<  MapPoints[i].Observations.size() << "\n";
            for (int j = 0; j < MapPoints[i].Observations.size(); j++)
            {
                ofs << j << " " << MapPoints[i].Observations[j].FrameID << " " << MapPoints[i].Observations[j].FeaIndex << "\n" ;
            }
            ofs << "WorldPos " << MapPoints[i].WorldPos(0) << " " << MapPoints[i].WorldPos(1) << " " << MapPoints[i].WorldPos(2) << "\n";
            ofs << "\n";    
        }
    }
    ofs.close();

    std::ofstream ofs2(Mppath + "/kFrames.txt");
    if (!ofs)
    {
        cout << " open fail Frames " << endl;
        return;
    }
    ofs2.clear();
    for (int i = 0; i < Frames.size(); i++)
    {
        ofs2 << "Frame " << Frames[i].id <<" " << Frames[i].info << "\n" ;
        ofs2 << "Pose\n";
        for (int x = 0; x < 4; x++)
        {
            for (int y = 0; y < 4; y++)
            {
                ofs2 << Frames[i].pose(x,y) << " ";
            }
            ofs2 << "\n";
        }
        ofs2 << "FeaPoints" << "\n";
        for (int j = 0; j < Frames[i].FeaPoints.size(); j++)
        {   
            ofs2 << j << " ";
            ofs2 << Frames[i].FeaPoints[j].id << " ";
            ofs2 << Frames[i].FeaPoints[j].p.x << " ";
            ofs2 << Frames[i].FeaPoints[j].p.y << " ";
            ofs2 << Frames[i].FeaPoints[j].dis << "\n";
        }
        ofs2 << "\n";
        
    }
    ofs2.close();

    //新的地图点和关键帧容器
    vector<MapPoint> GoodMapPoints;
    vector<kFrame> GoodFrames;
    GoodFrames.resize(Frames.size());
    //从上述观测和地图点中提取大于三次的观测并保存
    int Index = 0; // 用于保存新mp的Id
    //保pose
    for (int i = 0; i < Frames.size(); i++)
    {
        GoodFrames[i].pose = Frames[i].pose;
        GoodFrames[i].id = Frames[i].id;
        GoodFrames[i].info = Frames[i].info;
        GoodFrames[i].Near = Frames[i].Near;
        GoodFrames[i].timestamp = Frames[i].timestamp;
    }
    
    for (int i = 0; i < MapPoints.size(); i++)
    {
        int ob_num = MapPoints[i].Observations.size();
        if(ob_num >= 3 || MapPoints[i].Is_good)
        {
            //筛选出观测次数大于等于３的点 或　不错的帧间匹配点
            MapPoint mp;
            mp.id = Index;
            mp.desc = MapPoints[i].desc.clone();
            mp.WorldPos = MapPoints[i].WorldPos;
            mp.Is_good = true;
            for (int j = 0; j < ob_num; j++)
            {
                int frame_id = MapPoints[i].Observations[j].FrameID;
                int fea_index = MapPoints[i].Observations[j].FeaIndex;
                // New Feapoint
                FeaPoint fp;
                fp.p = Frames[frame_id].FeaPoints[fea_index].p;
                fp.dis = Frames[frame_id].FeaPoints[fea_index].dis;
                fp.id = mp.id;
                fp.Frame_id = frame_id;

                //Mp Obs
                Observation ob;
                ob.FrameID = frame_id;
                ob.FeaIndex = GoodFrames[frame_id].FeaPoints.size();
                GoodFrames[frame_id].FeaPoints.push_back(fp);
                mp.Observations.push_back(ob);
            }

            GoodMapPoints.push_back(mp);
            Index ++;
        }
    }

    std::ofstream ofs3(Mppath + "/GoodMapPoints.txt");
    if (!ofs3)
    {
        cout << " open fail GoodMappoints " << endl;
        return;
    }
    ofs3.clear();
    for (int i = 0; i < GoodMapPoints.size(); i++)
    {
        // 1. Id 
        int obnum= GoodMapPoints[i].Observations.size();
        if(obnum >=3|| GoodMapPoints[i].Is_good)
        {
            ofs3 << "Id ";
            ofs3 <<  GoodMapPoints[i].id << "\n";
            ofs3 << "ObsNum " ;
            ofs3 <<  GoodMapPoints[i].Observations.size() << "\n";
            for (int j = 0; j < GoodMapPoints[i].Observations.size(); j++)
            {
                ofs3 << j << " " << GoodMapPoints[i].Observations[j].FrameID << " " << GoodMapPoints[i].Observations[j].FeaIndex << "\n" ;
            }
            ofs3 << "WorldPos " << GoodMapPoints[i].WorldPos(0) << " " << GoodMapPoints[i].WorldPos(1) << " " << GoodMapPoints[i].WorldPos(2) << "\n";
            ofs3 << "\n";    
        }
    }
    ofs3.close();

    std::ofstream ofs4(Mppath + "/GoodFrames.txt");
    if (!ofs4)
    {
        cout << " open fail GoodFrames " << endl;
        return;
    }
    ofs4.clear();
    for (int i = 0; i < GoodFrames.size(); i++)
    {
        ofs4 << "Frame " << GoodFrames[i].id;
        // int a = 1,b = 0;
        // if(GoodFrames[i].Near == true)
        //     ofs4 << " " << a;
        // else
        //     ofs4 << " " << b; 
        ofs4  << " "<<  GoodFrames[i].info<< "\n" ;

        ofs4 << "Time" << " " << GoodFrames[i].timestamp << "\n" ;
        ofs4 << "Pose\n";
        for (int x = 0; x < 4; x++)
        {
            for (int y = 0; y < 4; y++)
            {
                ofs4 << GoodFrames[i].pose(x,y) << " ";
            }
            ofs4 << "\n";
        }
        ofs4 << "FeaNum " << GoodFrames[i].FeaPoints.size() << "\n";
        for (int j = 0; j < GoodFrames[i].FeaPoints.size(); j++)
        {   
            ofs4 << j << " ";
            ofs4 << GoodFrames[i].FeaPoints[j].id << " ";
            ofs4 << GoodFrames[i].FeaPoints[j].p.x << " ";
            ofs4 << GoodFrames[i].FeaPoints[j].p.y << " ";
            ofs4 << GoodFrames[i].FeaPoints[j].dis << "\n";
        }
        ofs4 << "\n";
        
    }
    ofs4.close();

    std::ofstream ofs5(Mppath + "/LidarNearFrame.txt");
    if (!ofs5)
    {
        cout << " open fail GoodFrames " << endl;
        return;
    }
    ofs5.clear();
    for (int i = 0; i < LidarNearFrame.size(); i++)
    {
        ofs5 << LidarNearFrame[i] << "\n" ;
    }
    ofs5.close();
    cout << " Write txt success!" << endl;
}


//该函数用于新特征匹配
void StereoCamera::ComputeMatch_Ransac(std::vector<cv::KeyPoint> Last_KP,cv::Mat Last_Desc,std::vector<cv::KeyPoint> KP,cv::Mat Desc,kFrame frame,kFrame frame_last)
{
    const size_t last_fnum = Last_KP.size();
    const size_t now_fnum = KP.size();
    std::vector<cv::DMatch> matches_bf;//用于存储暴力检测
    
    int width = ImageSize.width;
    int height = ImageSize.height;
    for (size_t ii = 0; ii < last_fnum; ii++)
    {
        int bestdist = 128;
        int bestidx = 0;
        cv::DMatch m;
        const cv::Mat &dL = Last_Desc.row(ii);
        for (size_t jj = 0; jj < now_fnum; jj++)
        {
            const cv::Mat &dN = Desc.row(jj);
            int dist = ComputeDist(dL,dN);
            if (dist < bestdist)
            {
                // std::cout << "dist = " <<dist << std::endl;
                bestdist = dist;
                bestidx = jj;
            }
        }
        m.distance = bestdist;
        m.queryIdx = ii;
        m.trainIdx = bestidx;
        matches_bf.push_back(m);
    }

    // std::vector<cv::DMatch> good_match;

    std::vector<PointInfo> Points;
    std::vector<cv::Point2f> Puv;
    std::vector<cv::Point3f> P3d;

    Eigen::Matrix4d dR;

    dR = Frames[NumofFrame-1].pose.inverse()*Pose;

    // std::cout << "dR_w :" << std::endl << dR << std::endl;
    dR = Tbc_inv*dR*Tbc;
    // std::cout << "dR_c :" << std::endl << dR << std::endl;
    int gotNUM = 0;
    for (size_t i = 0; i < matches_bf.size(); i++)
    {
        //对于好的特征匹配，计算视差
        //左目划窗设置为11
        int w = 5;
        const int &uL = KP[matches_bf[i].trainIdx].pt.x;
        const int &vL = KP[matches_bf[i].trainIdx].pt.y;
        if (uL-w-1 <0 || uL+w+1 > width)
            continue;//过界不要
        else if (vL-w-1<0 || vL+w+1 >height)
            continue;
        
        cv::Mat IL = Left_reIm.rowRange(vL-w,vL+w+1).colRange(uL-w,uL+w+1);
        IL.convertTo(IL,CV_32F);

        int bestDist = __INT_MAX__;
        int bestincR = 0;

        for (int incR = w+1; incR < width-w-2; incR++)
        {
            // ROS_INFO("u : %d,%d",incR-w,incR+w+1);
            cv::Mat IR = Right_reIm.rowRange(vL-w,vL+w+1).colRange(incR-w,incR+w+1);
            IR.convertTo(IR,CV_32F);
            float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
            if (dist < bestDist)
            {
                bestDist = dist;
                bestincR = incR;
            }
        }
        int d = uL-bestincR;
        if (d > 20 && d<400)
        {   // good_match.push_back(matches_bf[i]);
            
            //计算投影到上一帧的重投影误差
            Eigen::Vector3d puv((double)uL,(double)vL,1.0);
            Eigen::Vector3d Pc = ((double)bf/(double)d)*Left_Kinv*puv;
            // Eigen::Matrix4d dR;
            double Error = ErrorReProj(Last_KP[matches_bf[i].queryIdx].pt,dR,Pc);
            if(Error < RePerTH)
            {
                // std::cout << "******Error_ransac = " << Error << std::endl;
                gotNUM ++;
                //保存3D点【当前】 和 像素坐标【上一帧】

                //当前帧 三维点

                PointInfo p_if;
                p_if.p.x = uL;
                p_if.p.y = vL;
                p_if.Desc = Desc.row(matches_bf[i].trainIdx);
                p_if.Pc = Pc;
                Points.push_back(p_if);
                cv::Point3f p3d;
                p3d.x = (double)Pc(0);
                p3d.y = (double)Pc(1);
                p3d.z = (double)Pc(2);
                // std::cout << "P = " << p3d.x << " " << p3d.y << " " << p3d.z << std::endl;
                P3d.push_back(p3d);

                cv::Point2f uv;
                uv.x = (float)Last_KP[matches_bf[i].queryIdx].pt.x;
                uv.y = (float)Last_KP[matches_bf[i].queryIdx].pt.y;
                Puv.push_back(uv);

                // FeaPoint fp;
                // fp.p.x = uL;
                // fp.p.y = vL;
                // fp.id = NumofFea++;
                // fp.desc = Desc.row(matches_bf[i].trainIdx);
                // fp.dis = (double)d;
                // frame.FeaPoints.push_back(fp);

                // //同时也要获得上一帧的匹配情况
                // FeaPoint fp_last;
                // fp_last.p.x = Last_KP[matches_bf[i].queryIdx].pt.x;
                // fp_last.p.y = Last_KP[matches_bf[i].queryIdx].pt.y;
                // fp_last.id = fp.id;
                // fp_last.dis = (double)d;
                // frame_last.FeaPoints.push_back(fp_last);
            }
        }
    }
    // std:cout << "I got NUM = " << gotNUM << std::endl;

    cv::Mat distCoeffs;
    distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
	for (int i = 0; i < 3; i++)
	distCoeffs.at<double>(i, 0) = 0;

    Eigen::Matrix4d dR_inv4;
    dR_inv4 = dR.inverse();

    // std::cout << "dR inv : " << std::endl << dR_inv4 << std::endl;

    Eigen::Matrix3d dR_inv = dR_inv4.block(0,0,3,3);
    cv::Mat3f Rotation;
    cv::eigen2cv(dR_inv,Rotation);
    cv::Mat r;
    cv::Rodrigues(Rotation,r);
    cv::Mat t;
    t.create(3, 1, CV_64FC1);
    t.at<double>(0, 0) = (double)dR_inv4(0,3);
    t.at<double>(1, 0) = (double)dR_inv4(1,3);
    t.at<double>(2, 0) = (double)dR_inv4(2,3);
    cv::Mat Inliers;
    std::cout << "*********************************" << std::endl;
    std::cout << "*********************Input******" << std::endl;

    std::cout << "rvec = " << r.at<double>(0, 0) << " " << r.at<double>(1, 0) << " " << r.at<double>(2, 0) << std::endl;
    std::cout << "tvec = " << t.at<double>(0, 0) << " " << t.at<double>(1, 0) << " " << t.at<double>(2, 0) << std::endl;
    // cv::solvePnPRansac(P3d,Puv,Left_K, NULL,r,t,true,Inliers);

    cv::solvePnPRansac(P3d,Puv,Left_K,distCoeffs,r,t,false,30,8.0,0.9899999999999999911,Inliers,0);

    std::cout << "**********************Onput*****" << std::endl;
    std::cout << "****************************************************************************Inliers size = " << Inliers.size() << std::endl;
    std::cout << "rvec = " << r.at<double>(0, 0) << " " << r.at<double>(1, 0) << " " << r.at<double>(2, 0) << std::endl;
    std::cout << "tvec = " << t.at<double>(0, 0) << " " << t.at<double>(1, 0) << " " << t.at<double>(2, 0) << std::endl;
    std::cout << " Ransac! "<< std::endl;
    std::cout << "*********************************" << std::endl;
    cv::waitKey(500);
}

float ComputeDistSIFT(cv::Mat d1,cv::Mat d2)
{
    float dist = 0;
    for (int i = 0; i < 128; i++)
    {
        dist += abs(d1.at<float>(0,i)-d2.at<float>(0,i));
        // cout << dist << endl;
    }
    dist = dist/128.0;
    return dist;
}
void StereoCamera::Tracking2()
{
    ReMap();
    cv::Mat disp;
    ImageSize = Left_Im.size();
    // cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create();
    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(0,3,0.04,24,1.6);
    std::vector<cv::KeyPoint> vLeft_Kp;
    cv::Mat Left_Desc;

    int height = Left_Im.size().height;
    int width = Left_Im.size().width;
    
    // SIFT特征点和关键帧
    
    detector->detect(Left_reIm,vLeft_Kp);
    detector->compute(Left_reIm,vLeft_Kp,Left_Desc);

    // cv::Mat desc;
    // ORB_SLAM2::ORBextractor ob(1500,1.2,8,20,7);
    // ob.GetFea(Left_reIm,vLeft_Kp,desc);
    // // Left_Desc = desc.clone();   
    // detector->compute(Left_reIm,vLeft_Kp,Left_Desc);
    
    // vector<cv::KeyPoint> keypoints;
    // keypoints = OctTree(vLeft_Kp,height,width,700);
    // vLeft_Kp.clear();
    // for (int ki = 0; ki < keypoints.size(); ki++)
    // {
    //     vLeft_Kp.push_back(keypoints[ki]);
    // }
    

    kFrame ThisFrame;
    std::vector<cv::KeyPoint> KP_new;
    cv::Mat Desc_new;

    //计算当前帧的深度 使用SGBM
    cv::Mat Disp;
    ComputeDispAndReProj(Disp);

    if (NumofFrame == 0)
    {
        //只将图像传递给上一帧 并将关键帧和描述子保留
        Left_lastreIm = Left_reIm.clone();
        vLeft_lastKp.clear();
        for (int i = 0; i < vLeft_Kp.size(); i++)
            vLeft_lastKp.push_back(vLeft_Kp[i]);
        Left_lastDesc = Left_Desc.clone();
        Last_Disp = Disp.clone();
        ThisFrame.pose = Pose;
        PnPPose = Pose;
        ThisFrame.id = NumofFrame;
        ThisFrame.info = 0.1;
        ThisFrame.timestamp = Left_Time;
        Frames.push_back(ThisFrame);
        NumofFrame ++;
        return;
    }

    
    std::vector<cv::DMatch> matches_bf;//用于存储暴力检测
    Eigen::Matrix4d dR = dR = Frames[NumofFrame-1].pose.inverse()*Pose;
    dR = Tbc_inv*dR*Tbc;
    // 一些阈值
    const double z_max = 24;
    const double z_min = 0.8;
    const int Fea_winsize = 40;
        
    // 重投影当前帧特征点到上一帧进行局部BF搜索
    for (int fi = 0; fi < vLeft_Kp.size(); fi++)
    {
        // 提取视差并计算深度值
        double z = 16.0*bf/(double)Disp.at<unsigned short>(vLeft_Kp[fi].pt);
        // 剔除非正常深度值
        if( z < z_min || z>z_max)
            continue;
        
        double d = (double)Disp.at<unsigned short>(vLeft_Kp[fi].pt)/16.0;
        int u_current = (int)vLeft_Kp[fi].pt.x;
        int v_current = (int)vLeft_Kp[fi].pt.y;
        // 计算三维坐标点

        //使用Ｋ进行计算
        // Eigen::Vector3d puv((double)vLeft_Kp[fi].pt.x,(double)vLeft_Kp[fi].pt.y,1.0);
        // Eigen::Vector3d Pc = z*Left_Kinv*puv;
        //　使用Ｑ进行计算

        // Eigen::Vector4d Pc_;
        // Pc_(0) = Pc(0);
        // Pc_(1) = Pc(1);
        // Pc_(2) = Pc(2);
        // Pc_(3) = 1;
        Eigen::Vector4d Pc_;
        double W = Q.at<double>(3,2)*d*1000.0;
        Pc_(0) = ((double)u_current+Q.at<double>(0,3))/W;
        Pc_(1) = ((double)v_current+Q.at<double>(1,3))/W;
        Pc_(2) = Q.at<double>(2,3)/W;
        Pc_(3) = 1;
        Eigen::Vector4d Pr_ = dR*Pc_;
        Eigen::Vector3d Pr;
        Pr(0) = Pr_(0);
        Pr(1) = Pr_(1);
        Pr(2) = Pr_(2);
        
        double s = Q_inv.at<double>(3,2)*Pr(2);
        int u_reproj = (Pr(0)+Pr(2)*Q_inv.at<double>(0,2))/s;
        int v_reproj = (Pr(1)+Pr(2)*Q_inv.at<double>(1,2))/s;

        // int u_reproj = round(Left_K.at<double>(0,0)*Pr(0)/Pr(2) + Left_K.at<double>(0,2));
        // int v_reproj = round(Left_K.at<double>(1,1)*Pr(1)/Pr(2) + Left_K.at<double>(1,2));
        
        // 剔除图像范围外的点
        if(u_reproj < 0 || v_reproj <0 || u_reproj > width || v_reproj > height)
        {
            continue;
        }
        // 保存当前帧该特征的描述子
        const cv::Mat &d_current = Left_Desc.row(fi);
        // 设置窗口边界
        int u_max = u_reproj + Fea_winsize;
        if( u_max > width)
            u_max = width-1;
        int u_min = u_reproj - Fea_winsize;
        if( u_min < 0)
            u_min = 1;
        int v_max = v_reproj + Fea_winsize;
        if( v_max > height)
            v_max = height-1;
        int v_min = v_reproj - Fea_winsize;
        if( v_min < 0)
            v_min = 1;
        // 对在区域内的特征子进行暴力搜索最优匹配
        float bestdist = 128;
        int bestidx = 0;
        for (int fi_last = 0; fi_last < vLeft_lastKp.size(); fi_last++)
        {
            int u_last = vLeft_lastKp[fi_last].pt.x;
            int v_last = vLeft_lastKp[fi_last].pt.y;
            // 是否落在局部区域内
            if( (u_last < u_max) && (u_last > u_min) && (v_last < v_max) && (v_last > v_min) )
            {
                const cv::Mat &d_last = Left_lastDesc.row(fi_last);
                // int dist = ComputeDist(d_current,d_last);
                float dist =ComputeDistSIFT(d_current,d_last);
                if(dist < bestdist)
                {
                    bestdist = dist;
                    bestidx = fi_last;
                } 
            }    
        }
        double z2 = 16.0*bf/(double)Last_Disp.at<unsigned short>(vLeft_lastKp[bestidx].pt);
        if(bestdist < 4 && abs(z-z2) < 5)
        {
            // cout << "Bestdis = " << bestdist << endl;
            cv::DMatch m;
            m.distance = bestdist;
            m.queryIdx = fi;
            m.trainIdx = bestidx;
            matches_bf.push_back(m);
        }
    }

    // cout << "matches_bs = " << matches_bf.size() << endl;
    bool Ransac = true;

    cv::Mat im_ransac;
    vector<cv::DMatch> matcher_Ransac;
    if(Ransac)
    {
        cv::Mat p1(matches_bf.size(),2,CV_32F);
        cv::Mat p2(matches_bf.size(),2,CV_32F);
        for (int mi = 0; mi < matches_bf.size(); mi++)
        {
            p1.at<float>(mi,0) = vLeft_lastKp[matches_bf[mi].trainIdx].pt.x;
            p1.at<float>(mi,1) = vLeft_lastKp[matches_bf[mi].trainIdx].pt.y;
            p2.at<float>(mi,0) = vLeft_Kp[matches_bf[mi].queryIdx].pt.x;
            p2.at<float>(mi,1) = vLeft_Kp[matches_bf[mi].queryIdx].pt.y;
        }
        if(matches_bf.size() > 10)
        {
            cv::Mat mFundamental;
            vector<uchar> status;
            mFundamental = cv::findFundamentalMat(p1,p2,status,CV_FM_RANSAC);
            cv::Mat mHomography;
            vector<uchar> m;
            mHomography = cv::findHomography(p1,p2,CV_RANSAC,3,m);

            int Outliers = 0;
            for (int si = 0; si < status.size(); si++)
            {
                if(status[si] == 0){
                Outliers++;
                }
            }
            vector<cv::Point2f> Inlier_c,Inlier_l;
            int ptCount = matches_bf.size();
            int InlierCount = ptCount - Outliers;
            matcher_Ransac.resize(InlierCount);
            Inlier_c.resize(InlierCount);
            Inlier_l.resize(InlierCount);
            InlierCount = 0;
            for (int si = 0; si < status.size(); si++)
            {
                if(status[si] != 0)
                {
                    Inlier_c[InlierCount].x = vLeft_Kp[matches_bf[si].queryIdx].pt.x;
                    Inlier_c[InlierCount].y = vLeft_Kp[matches_bf[si].queryIdx].pt.y;
                    Inlier_l[InlierCount].x = vLeft_lastKp[matches_bf[si].trainIdx].pt.x;
                    Inlier_l[InlierCount].y = vLeft_lastKp[matches_bf[si].trainIdx].pt.y;
                    matcher_Ransac[InlierCount].queryIdx = matches_bf[si].queryIdx;
                    matcher_Ransac[InlierCount].trainIdx = matches_bf[si].trainIdx;
                    matcher_Ransac[InlierCount].distance = matches_bf[si].distance;
                    InlierCount++;
                }
            }
        }
        else
        {
            matcher_Ransac.clear();
        }

        if(matcher_Ransac.size() <= 10)
            matcher_Ransac.clear();
        cout << "  Fea  Num = " << matches_bf.size() << endl;
        cout << "Ransac Num = " << matcher_Ransac.size() << endl;
    }


    //　每次匹配选择１５个好匹配用以保留
    double distant[matcher_Ransac.size()];
    for (int fi = 0; fi < matcher_Ransac.size(); fi++)
    {
        distant[fi] = matcher_Ransac[fi].distance;
    }
    sort(distant,distant+matcher_Ransac.size());
    double gooddis;
    if(matcher_Ransac.size() <= 10)
    {
            gooddis = 10.0;
    }
    else
    {
        gooddis = distant[10];
    }
    cout << "dis good  = " << gooddis << endl;

    for (int fi = 0; fi < matcher_Ransac.size(); fi++)
    {   
        Eigen::Vector4d Pw;
        int index = matcher_Ransac[fi].queryIdx;

        // double z = 16.0*bf/(double)Disp.at<unsigned short>(vLeft_Kp[index].pt);
        // Eigen::Vector3d puv((double)vLeft_Kp[index].pt.x,(double)vLeft_Kp[index].pt.y,1.0);
        // Eigen::Vector3d Pc = z*Left_Kinv*puv;
        // Pw(0) = Pc(0);
        // Pw(1) = Pc(1);
        // Pw(2) = Pc(2);
        // Pw(3) = 1;

        double u_c = (double)vLeft_Kp[index].pt.x;
        double v_c = (double)vLeft_Kp[index].pt.y;
        double d_c = (double)Disp.at<unsigned short>(vLeft_Kp[index].pt)/16.0;

        double W = Q.at<double>(3,2)*d_c*1000.0;
        Pw(0) = (u_c+Q.at<double>(0,3))/W;
        Pw(1) = (v_c+Q.at<double>(1,3))/W;
        Pw(2) = Q.at<double>(2,3)/W;
        Pw(3) = 1;

        Pw = Pose*Tbc*Pw;
        //find nearest point
        //检查
        bool IsNew = true;

        //冲投影误差
        int min_e = 1000000;
        int min_midx = 0;
        for (int mi = 0; mi < MapPoints.size(); mi++)
        {
            // 初步筛选提高效率
            if(abs(MapPoints[mi].WorldPos(0)-Pw(0))>2)
                continue;
            Eigen::Vector4d Pmi_w;
            Pmi_w << MapPoints[mi].WorldPos(0),MapPoints[mi].WorldPos(1),MapPoints[mi].WorldPos(2),1;
            Eigen::Vector4d Pmi_c;
            Pmi_c = (Pose*Tbc).inverse()*Pmi_w;
            // int u_reproj = round(Left_K.at<double>(0,0)*Pmi_c(0)/Pmi_c(2) + Left_K.at<double>(0,2));
            // int v_reproj = round(Left_K.at<double>(1,1)*Pmi_c(1)/Pmi_c(2) + Left_K.at<double>(1,2));
            double s = Q_inv.at<double>(3,2)*Pmi_c(2);
            int u_reproj = (Pmi_c(0)+Pmi_c(2)*Q_inv.at<double>(0,2))/s;
            int v_reproj = (Pmi_c(1)+Pmi_c(2)*Q_inv.at<double>(1,2))/s;

            int error = abs(u_reproj-u_c) + abs(v_reproj-v_c);
            if(error < min_e)
            {
                min_e = error;
                min_midx = mi;
            }
        }
        //
        // cout << "min_dis = " << min_pd << " index = " << min_midx << endl;
        // if(min_pd < 0.2*z)
        // cout << "Min error = " << min_e << endl;
        if(min_e < 10)
        {
            cv::Mat dM = MapPoints[min_midx].desc;
        // cout << "!" << endl;
            cv::Mat dF = Left_Desc.row(index);
            float descdist = ComputeDistSIFT(dM,dF);
            if(descdist < 3.6 && MapPoints[min_midx].newest_frame != NumofFrame)
            {
                // cout << "*********************" << endl;
                // cout << "Find a same point :   dist = " << min_pd << " Descdist = " << descdist <<endl;
                // cout << " fi = " << fi << " from Frame = " << MapPoints[min_midx].newest_frame << endl;
                IsNew = false;
                Observation ob;
                int F_id = MapPoints[min_midx].newest_frame;
                ob.FrameID = NumofFrame;
                ob.FeaIndex = ThisFrame.FeaPoints.size();
                MapPoints[min_midx].Observations.push_back(ob);
                FeaPoint fp;
                fp.id = MapPoints[min_midx].id;
                fp.desc = dF;
                fp.dis = (double)vLeft_Kp[index].pt.x - (double)Disp.at<unsigned short>(vLeft_Kp[index].pt)/16.0;
                fp.Frame_id = NumofFrame;
                fp.p = vLeft_Kp[index].pt;
                ThisFrame.FeaPoints.push_back(fp);

                // //更新Pos 和　Desc
                // MapPoints[min_midx].WorldPos(0) = (MapPoints[min_midx].WorldPos(0)+Pw(0))/2;
                // MapPoints[min_midx].WorldPos(1) = (MapPoints[min_midx].WorldPos(1)+Pw(1))/2;
                // MapPoints[min_midx].WorldPos(2) = (MapPoints[min_midx].WorldPos(2)+Pw(2))/2;

                // for (int di = 0; di < di; di++)
                // {
                //     MapPoints[min_midx].desc.at<unsigned short>(0,di) = (MapPoints[min_midx].desc.at<unsigned short>(0,di)+dF.at<unsigned short>(0,di))/2;
                // }
                
            }
        }

        // 新观测
        // cout << "Min d = " << min_pd << endl;
        if (IsNew)
        {
            MapPoint mp;
            mp.WorldPos(0) = Pw(0);
            mp.WorldPos(1) = Pw(1);
            mp.WorldPos(2) = Pw(2);
            // mp.desc = 
            mp.id = NumofFea++;
            mp.desc = Left_Desc.row(index);
            mp.newest_frame = NumofFrame;
            mp.Is_good = false;
            
            Observation ob;
            ob.FrameID = NumofFrame;
            ob.FeaIndex = ThisFrame.FeaPoints.size();
            Observation ob_last;
            ob_last.FrameID = NumofFrame -1;
            ob_last.FeaIndex = Frames[NumofFrame-1].FeaPoints.size();
            mp.Observations.push_back(ob_last);
            mp.Observations.push_back(ob);
            
            //增加观测 本帧观测
            FeaPoint fp;
            fp.id = mp.id;
            fp.Frame_id = NumofFrame;
            fp.desc = mp.desc;
            fp.dis = (double)vLeft_Kp[index].pt.x - (double)Disp.at<unsigned short>(vLeft_Kp[index].pt)/16.0;
            fp.p = vLeft_Kp[index].pt;
            ThisFrame.FeaPoints.push_back(fp);
            //增加观测，上一帧观测
            FeaPoint fp_last;
            fp_last.id = mp.id ;
            fp_last.Frame_id = NumofFrame - 1;
            fp_last.desc = Left_lastDesc.row(matcher_Ransac[fi].trainIdx);
            fp_last.dis = (double)vLeft_lastKp[matcher_Ransac[fi].trainIdx].pt.x-(double)Last_Disp.at<unsigned short>(vLeft_lastKp[matcher_Ransac[fi].trainIdx].pt)/16.0;
            fp_last.p = vLeft_lastKp[matcher_Ransac[fi].trainIdx].pt;
            Frames[NumofFrame-1].FeaPoints.push_back(fp_last);
            if(matcher_Ransac[fi].distance < 2.5 && matcher_Ransac[fi].distance < gooddis)
            {
                mp.Is_good = true;
            }
            MapPoints.push_back(mp);
        }
    }


    if(true)
    {
        cv::Mat imshow;
        cv::vconcat(Left_lastreIm,Left_reIm,imshow);
        for (int fi = 0; fi < matcher_Ransac.size(); fi++)
        {
            cv::Point2i pc,pl;
            pc.x = vLeft_Kp[matcher_Ransac[fi].queryIdx].pt.x;
            pc.y = vLeft_Kp[matcher_Ransac[fi].queryIdx].pt.y + height;
            pl.x = vLeft_lastKp[matcher_Ransac[fi].trainIdx].pt.x;
            pl.y = vLeft_lastKp[matcher_Ransac[fi].trainIdx].pt.y;
            cv::line(imshow,pl,pc,CV_RGB(0,255,0),1);
            cv::circle(imshow,pl,2,CV_RGB(255,0,0),2);
            cv::circle(imshow,pc,2,CV_RGB(255,0,0),2);
        }
        cv::imshow("im",imshow);
        cv::waitKey(1);
    }


    Left_lastlastDesc = Left_lastDesc.clone();
    Left_lastlastreIm = Left_lastreIm.clone();
    vLeft_lastlastKp.clear();
    
    for (int vi = 0; vi < vLeft_lastKp.size(); vi++)
        vLeft_lastlastKp.push_back(vLeft_lastKp[vi]);
    last_match.clear();
    
    last_match.resize(matcher_Ransac.size());
    for (int mi = 0; mi < matcher_Ransac.size(); mi++)
    {
        
        last_match[mi].queryIdx = matcher_Ransac[mi].queryIdx;
        last_match[mi].trainIdx = matcher_Ransac[mi].trainIdx;
    }
   

    std::cout << "*************************************" <<std::endl;
    std::cout << " Frame  " <<  NumofFrame << " done!" <<std::endl;

    Left_lastreIm = Left_reIm.clone();
    vLeft_lastKp.clear();
    for (int i = 0; i < vLeft_Kp.size(); i++)
        vLeft_lastKp.push_back(vLeft_Kp[i]);
    Left_lastDesc = Left_Desc.clone();
    Last_Disp = Disp.clone();

    ThisFrame.pose = Pose;
    ThisFrame.id = NumofFrame;
    ThisFrame.timestamp = Left_Time;
    ThisFrame.info = (double)matcher_Ransac.size()/(10*gooddis*gooddis);
    if(ThisFrame.info < 0.1)
        ThisFrame.info = 0.1;
    // cout << " w = " << ThisFrame.info << endl;
    Frames.push_back(ThisFrame);
    NumofFrame ++;
}

}