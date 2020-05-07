#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <visual_fea_test/StereoFrame.hpp>
#include <pangolin/pangolin.h>
#include <opencv2/core/eigen.hpp>
using namespace std;

struct Lidarpose{
    int id;
    Eigen::Matrix4d pose;
    Eigen::Quaterniond q;
    uint64_t timestamp_sec;
    uint64_t timestamp_nsec;
};
void GetImage(sensor_msgs::ImageConstPtr Im_msgP,cv::Mat & Im, bool &flag,ros::Time &time);

void GetLidarPose(string posepath,vector<Lidarpose> &LidarPose_);

void GetFileNames(string path,vector<string>& filenames);

void GetCamTfromLidar(ros::Time time,vector<Lidarpose> LidarPose,Eigen::Matrix4d &CamT);

void DrawTrajectory(vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses,vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_pnp,std::vector<Stereo::MapPoint> MPs);


int main(int argc, char **argv){
    
    ros::init(argc, argv, "GraduationProject");

    ros::NodeHandle nh;
    
    int a,b;
    nh.getParam("startFrame", a);
    nh.getParam("endFrame", b);
    
    // 读取雷达pose
    std::string posepath;
    nh.getParam("posepath", posepath);
    vector<Lidarpose> LidarPose;
    GetLidarPose(posepath,LidarPose);

    // 读取相机参数
    std::string yamlpath,txtpath_mp,txtpath_kf;
    nh.getParam("yamlpath", yamlpath);
    nh.getParam("txtpath_mp", txtpath_mp);
    nh.getParam("txtpath_kf", txtpath_kf);
    Stereo::StereoCamera camera ;
    camera.GetParm(yamlpath);

    // 从rosbag读取Topic
    std::string bagpath,tLeft,tRight;
    nh.getParam("bagpath", bagpath);
    nh.getParam("LeftImage_topic", tLeft);
    nh.getParam("RightImage_topic", tRight);
    //可视化参数
    bool DrawTraj,ShowNum,SavePcd,WriteTxt;
    nh.getParam("DrawTraj", DrawTraj);
    nh.getParam("ShowNum", ShowNum);
    nh.getParam("SavePcd",SavePcd);
    nh.getParam("WriteTxt",WriteTxt);
    nh.getParam("RePerTH",camera.RePerTH);
    nh.getParam("UseRansac",camera.UseRansac);
    rosbag::Bag bag;
    double sum_rTH,sum_tTH;
    nh.getParam("sum_rTH",sum_rTH);
    nh.getParam("sum_tTH",sum_tTH);

    bag.open(bagpath);
    ROS_INFO("Begin to read rosbag ...");
    bool Left_Image_Get = false,Right_Image_Get = false;
    cv::Mat left_image,right_image;
    ros::Time LeftTime,RightTime;


    int i = 0;
    double dt_now =1 ,dt_last = 1;
    int j = 0;
    //用于pangolin可视化
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses; 
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_pnp; 
    vector<string> Left_path;
    vector<string> Right_path;
    vector<string> pic_path;
    vector<cv::Mat> Left;
    vector<cv::Mat> Right;

    cv::Mat p1,p2;
    Stereo::kFrame f1,f2;
    bool first;
    first = true;
    // vector<int> NearLidarFrameId;
    vector<int64> time_stamp;
    for(rosbag::MessageInstance const m: rosbag::View(bag)){
        if (m.getTopic() == tLeft )
            GetImage(m.instantiate<sensor_msgs::Image>(),camera.Left_Im,Left_Image_Get,camera.Left_Time);
        if (m.getTopic() == tRight)
            GetImage(m.instantiate<sensor_msgs::Image>(),camera.Right_Im,Right_Image_Get,camera.Right_Time);
        if (Left_Image_Get && Right_Image_Get)
        {
            i++;
            if (i>a && i<b)
            {
                
                double time_c = (double)camera.Left_Time.toNSec()/1000000000;
                double time_l = (double)LidarPose[j].timestamp_sec + (double)LidarPose[j].timestamp_nsec/1000000000;
                dt_now = time_l - time_c;

                Eigen::Matrix4d CamT;
                GetCamTfromLidar(camera.Left_Time,LidarPose,CamT);
                camera.Pose = CamT;
                camera.R = CamT.block(0,0,3,3);
                camera.t = Eigen::Vector3d(CamT(0,3),CamT(1,3),CamT(2,3));
                Eigen::Matrix4d dT;
                dT = camera.Pose_last.inverse()*CamT;
                cv::Mat R;
                Eigen::Matrix3d R_e = dT.block(0,0,3,3);
                cv::eigen2cv(R_e,R);
                cv::Vec3d r;
                cv::Rodrigues(R,r);
                double sum_r = abs(r(0)) +abs(r(1))+abs(r(2));
                double sum_t = abs(dT(0,3)) +abs(dT(1,3))+abs(dT(2,3));

                if(first){
                    first = false;
                    camera.Pose_last = camera.Pose;
                }

                if(dt_last*dt_now < 0)
                {
                    j++;
                    camera.NearLidar = true;
                    camera.Tracking2();
                    int64 time =  (double)camera.Left_Time.toNSec();
                    time_stamp.push_back(time);
                    cv::imwrite("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/Left/" + to_string(time) +".jpg",camera.Left_reIm);
                    cv::imwrite("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/Right/" + to_string(time) +".jpg",camera.Right_reIm);
                    camera.Frames[camera.NumofFrame-1].Near = true;           
                    camera.LidarNearFrame.push_back(camera.NumofFrame-1);        
                    camera.Pose_last = camera.Pose; 
                    //×××××××××××××××××××××××××××××××××××××××××
                    // from T get Pose  用于使用pangolin可视化
                    if (DrawTraj)
                    {
                        Eigen::Matrix3d R;
                        R = CamT.block(0,0,3,3);
                        Eigen::Quaterniond Q(R);
                        Eigen::Isometry3d Twr(Eigen::Quaterniond(Q.w(),Q.x(),Q.y(),Q.z()));
                        Twr.pretranslate(Eigen::Vector3d(CamT(0,3),CamT(1,3),CamT(2,3)));
                        poses.push_back(Twr);
                    }
                    dt_now = 1;
                }
                else if(sum_t > sum_tTH || sum_r > sum_rTH)
                {

                    camera.NearLidar = false;
                    camera.Tracking2(); 
                    int64 time =  (double)camera.Left_Time.toNSec();
                    
                    time_stamp.push_back(time);
                    cv::imwrite("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/Left/" + to_string(time) +".jpg",camera.Left_reIm);
                    cv::imwrite("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/Right/" + to_string(time) +".jpg",camera.Right_reIm);

                    camera.Frames[camera.NumofFrame-1].Near = false; 
                    camera.Pose_last = camera.Pose; 
                    //×××××××××××××××××××××××××××××××××××××××××
                    // from T get Pose  用于使用pangolin可视化
                    if (DrawTraj)
                    {
                        Eigen::Matrix3d R;
                        R = CamT.block(0,0,3,3);
                        Eigen::Quaterniond Q(R);
                        Eigen::Isometry3d Twr(Eigen::Quaterniond(Q.w(),Q.x(),Q.y(),Q.z()));
                        Twr.pretranslate(Eigen::Vector3d(CamT(0,3),CamT(1,3),CamT(2,3)));
                        poses.push_back(Twr);

                        Eigen::Matrix3d R_;
                        Eigen::Matrix4d T_ = CamT*camera.Tbc;
                        R_ = T_.block(0,0,3,3);
                        Eigen::Quaterniond Q_(R_);
                        Eigen::Isometry3d Twr_(Eigen::Quaterniond(Q_.w(),Q_.x(),Q_.y(),Q_.z()));
                        Twr_.pretranslate(Eigen::Vector3d(T_(0,3),T_(1,3),T_(2,3)));
                        poses_pnp.push_back(Twr_);
                    }
                }
                
                dt_last = dt_now;

            }
            Left_Image_Get = false;
            Right_Image_Get = false;
        }
       
    }

    if (WriteTxt)
    {
        camera.WriteMapPoints(txtpath_mp);
    }

    

    if (ShowNum)
    {
        int Num_all = 0;
        std::cout << "*****************************" << std::endl;
        std::cout << "Num of Frame : "<<camera.Frames.size() << std::endl;
        for (int fi = 0; fi < camera.Frames.size(); fi++)
        {
            Num_all += camera.Frames[fi].FeaPoints.size();
            // std::cout << " Frame " << camera.Frames[fi].id << "'s fea num = " <<camera.Frames[fi].FeaPoints.size() << std::endl;
        }
        std::cout << "Num of Fea : " << camera.NumofFea << std::endl; 
        std::cout << "Num of All : " << Num_all << std::endl; 

        std::cout << "Num of Mappoint : " << camera.MapPoints.size() << std::endl;

        int good_ob = 0;
        for (int fi = 0; fi < camera.MapPoints.size(); fi++)
        {
            if(camera.MapPoints[fi].Observations.size() > 2 || camera.MapPoints[fi].Is_good)
                good_ob ++;
        }
        cout << "Num of good Obs = " << good_ob << endl;
    }
        
    if (SavePcd)
        camera.GetPCL(camera.MapPoints,"/home/qk/Music/P1.pcd");
    
    if (DrawTraj)
        DrawTrajectory(poses,poses_pnp,camera.MapPoints);

    cout << " Begin to write txt ! ..." << endl;
    std::ofstream ofs("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/timestamp.txt");
    if (!ofs)
    {
        cout << " open fail Mappoints " << endl;
    }
    ofs.clear();
    for (int i = 0; i < time_stamp.size(); i++)
    {

        ofs << to_string(time_stamp[i]) << ".jpg" << endl;
    }
    ofs.close();
    cout << "Write timestamp.txt finfish!" << endl;

}

void GetImage(sensor_msgs::ImageConstPtr Im_msgP,cv::Mat & Im, bool &flag,ros::Time &time)
{
    time = Im_msgP->header.stamp;
    // uint64_t time_ = time.toNSec();
    // std::cout << "time 000 = " << time_ << std::endl;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*Im_msgP,sensor_msgs::image_encodings::BGR8);
    Im = cv_ptr->image;
    // cv::cvtColor(Im,Im,cv::COLOR_RGB2GRAY);
    flag = true;
}

void GetFileNames(string path,vector<string>& filenames)
{
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str()))){
        cout<<"Folder doesn't Exist!"<<endl;
        return;
    }
    while((ptr = readdir(pDir))!=0) {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0){
            filenames.push_back(path + "/" + ptr->d_name);
    }
    }
    closedir(pDir);
}

void GetLidarPose(string posepath,vector<Lidarpose> &LidarPose_){

    std::ifstream ifs(posepath);
    if (!ifs)
    {
        cout << " open fail " << endl;
    }
    
    vector<Lidarpose> LidarPose;
    
    vector<string> name;
    GetFileNames(posepath,name);

    for (int fi = 0; fi < name.size(); fi++)
    {
        string fd = name[fi] + "/data";
        ifstream fin(fd);
        if (fin)
        {
            string tb;
            Lidarpose lp;
            while (!fin.eof())
            {
                fin>>tb;
                double m[3][3];
                if (tb == "stamp")
                {
                    uint64_t t1,t2;
                    fin>>t1;
                    fin>>t2;
                    lp.timestamp_sec = t1;
                    lp.timestamp_nsec = t2;
                }
                else if(tb == "estimate")
                {
                    for (int i = 0; i < 4; i++)
                    {
                        for (int j = 0; j < 4; j++)
                        {
                            double p;
                            fin>>p;
                            lp.pose(i,j) = p;
                        }
                    }
                }
                else if(tb == "id"){
                    int id;
                    fin >> id;
                    lp.id = id;
                }
            }
            LidarPose.push_back(lp);
        }
    }

    for (int i = 0; i < LidarPose.size(); i++)
    {
        for (int j = 0; j < LidarPose.size(); j++)
        {
            if ( LidarPose[j].id == i)
            {
                LidarPose_.push_back(LidarPose[j]);
                
                // std::cout << "Lidar Pose " << i << std::endl << LidarPose[j].pose << std::endl;

            }    
        }        
    }
    ifs.close();
}

void GetCamTfromLidar(ros::Time time,vector<Lidarpose> LidarPose,Eigen::Matrix4d &CamT)
{
    double time_c = (double)time.toNSec()/1000000000;
    for (int j = 0; j < LidarPose.size()-1; j++)
    {
        double time_l1 = (double)LidarPose[j].timestamp_sec + (double)LidarPose[j].timestamp_nsec/1000000000;
        double time_l2 = (double)LidarPose[j+1].timestamp_sec + (double)LidarPose[j+1].timestamp_nsec/1000000000;
        if (time_l1 < time_c && time_l2 > time_c)
        {
            // ROS_INFO("**************");
            // ROS_INFO("time l1 = %f",time_l1);
            // ROS_INFO("time c = %f",time_c);
            // ROS_INFO("time l2 = %f",time_l2);
            double d = (time_c-time_l1)/(time_l2-time_l1);
            Eigen::Matrix4d T = LidarPose[j].pose;
            Eigen::Matrix4d T_next = LidarPose[j+1].pose;
            Eigen::Matrix4d dT =  T.inverse() * T_next ;
            Eigen::Matrix3d dR;
            // std::cout << "dT" << std::endl << dT<< std::endl;
            dR = dT.block(0,0,3,3);
            dT(0,3) = dT(0,3) * d;
            dT(1,3) = dT(1,3) * d;
            dT(2,3) = dT(2,3) * d;
            Eigen::AngleAxisd r(dR);
            Eigen::AngleAxisd r_d(r.angle()*d,r.axis());  //修正后的 相对 旋转向量
            Eigen::Matrix3d R_d;
            R_d = r_d.toRotationMatrix();
            for(int ii = 0;ii<3;ii++)
            {
                for(int jj = 0;jj<3;jj++)
                    dT(ii,jj) = R_d(ii,jj);
            }
            CamT = LidarPose[j].pose * dT;
            // CamT = LidarPose[j].pose;
            return;
        }
        
    }
}

void DrawTrajectory(vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses,vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses_pnp,std::vector<Stereo::MapPoint> MPs) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
    pangolin::ModelViewLookAt(0, -10, -0.1, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
    // std::cout << "draw!" << std::endl;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glLineWidth(3);
    // for (int ip = 0; ip < poses.size()-1; ip++) {
    //   // 画每个位姿的三个坐标轴
    //   if (ip%10 != 0)
    //   {
    //       continue;
    //   }
      
    //   Eigen::Vector3d Ow = poses[ip].translation();
    //   Eigen::Vector3d Xw = poses[ip] * (0.2 * Eigen::Vector3d(1, 0, 0));
    //   Eigen::Vector3d Yw = poses[ip] * (0.5 * Eigen::Vector3d(0, 1, 0));
    //   Eigen::Vector3d Zw = poses[ip] * (0.25 * Eigen::Vector3d(0, 0, 1));
    //   Eigen::Vector3d Xw_ = poses[ip] * (-0.2 * Eigen::Vector3d(1, 0, 0));
    //   Eigen::Vector3d Yw_ = poses[ip] * (-0.5 * Eigen::Vector3d(0, 1, 0));
    //   Eigen::Vector3d Zw_ = poses[ip] * (-0.25 * Eigen::Vector3d(0, 0, 1));
    //   glBegin(GL_LINES);
    // //   glColor3f(1.0, 0.0, 0.0);
    // //   glVertex3d(Ow[0], Ow[1], Ow[2]);
    // //   glVertex3d(-Xw[0], Xw[1], Xw[2]);
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Ow[0], Yw[1], Zw[2]);
    //     glVertex3d(Ow[0], Yw_[1], Zw[2]);
      
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Ow[0], Yw[1], Zw_[2]);
    //     glVertex3d(Ow[0], Yw_[1], Zw_[2]);
      
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Ow[0], Yw[1], Zw[2]);
    //     glVertex3d(Ow[0], Yw[1], Zw_[2]);

    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Ow[0], Yw_[1], Zw[2]);
    //     glVertex3d(Ow[0], Yw_[1], Zw_[2]);

    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Xw_[0], Xw[1], Xw[2]);
    //     glVertex3d(Ow[0], Yw_[1], Zw_[2]);
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Xw_[0], Xw[1], Xw[2]);
    //     glVertex3d(Ow[0], Yw_[1], Zw[2]);
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Xw_[0], Xw[1], Xw[2]);
    //     glVertex3d(Ow[0], Yw[1], Zw_[2]);
    //     glColor3f(0.0, 0.0, 1.0);
    //     glVertex3d(Xw_[0], Xw[1], Xw[2]);
    //     glVertex3d(Ow[0], Yw[1], Zw[2]);
    //     glEnd();
    // }
    // glPointSize(6);
    // glBegin(GL_POINTS);
    // for (size_t i = 0; i < poses.size(); i++)
    // {
    //     Eigen::Vector3d Ow = poses[i].translation();
    //     glColor3f(1.0, 0.0, 0.0);
    //     glVertex3d(Ow[0], Ow[1], Ow[2]);
    // }
    
    // 画出连线
    for (size_t ip = 0; ip < poses.size()-1; ip++) {
      glColor3f(0.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses[ip], p2 = poses[ip + 1];
      glVertex3d( 1 * p1.translation()[0], 1 * p1.translation()[1], 1 * p1.translation()[2]);
      glVertex3d( 1 * p2.translation()[0], 1 * p2.translation()[1], 1 * p2.translation()[2]);
      glEnd();
    }

    // 画出连线pnp
    for (size_t ip = 0; ip < poses_pnp.size()-1; ip++) {
      glColor3f(1.0, 0.0, 0.0);
      glBegin(GL_LINES);
      auto p1 = poses_pnp[ip], p2 = poses_pnp[ip + 1];
      glVertex3d( 1 * p1.translation()[0], 1 * p1.translation()[1], 1 * p1.translation()[2]);
      glVertex3d( 1 * p2.translation()[0], 1 * p2.translation()[1], 1 * p2.translation()[2]);
      glEnd();
    }

    glPointSize(3);
    glBegin(GL_POINTS);
    // 画出地图点
    for (size_t i = 0; i < MPs.size(); i++)
    {
        int o = MPs[i].Observations.size();
        if(o > 3)
        {
            
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(MPs[i].WorldPos(0),MPs[i].WorldPos(1),MPs[i].WorldPos(2));
        }
        else if( o > 2)
        {
            glColor3f(0.0, 0.0, 1.0);
            glVertex3d(MPs[i].WorldPos(0),MPs[i].WorldPos(1),MPs[i].WorldPos(2));
        }
        
        // else
        // {
        //     glPointSize(2);
        //     glColor3f(0.0, 0.0, 0.0);
        //     glVertex3d(MPs[i].WorldPos(0),MPs[i].WorldPos(1),MPs[i].WorldPos(2));
        // }
    }
    glEnd();
    pangolin::FinishFrame();
  }
}



