
#include <yaml-cpp/yaml.h>

#include "visual_fea_test/Optimizer.hpp"

using namespace std;
using namespace Stereo;
using namespace ORB_SLAM2;
typedef vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> Poses;

template <typename T>
void operator>>(const YAML::Node& node,T& i ){
    i = node.as<T>();
};

// int main(int argc, char **argv){

//     cout << "hello optimizer!" << endl;

//     YAML::Node node = YAML::LoadFile("/home/qk/Documents/dj2_MapPoint/Opti.yaml");
//     int FrameId,ShowOptiResult,ShowReproj,AddPlaneEdge,ShowPlane;
//     node["FrameId"] >> FrameId;
//     node["ShowOptiResult"] >> ShowOptiResult;
//     node["ShowReProj"] >> ShowReproj;
//     node["AddPlaneEdge"] >> AddPlaneEdge;
//     node["ShowPlane"] >> ShowPlane;

//     Eigen::Matrix4d Tlc;
//     Tlc << -0.0266583,    0.0145156,    0.999539,   -0.00581044,
//             -0.999635,   -0.000466258,   -0.0265932,    0.24939,
//                 0.000427441,   -0.999884,    0.0146246,   -0.16,
//                     0,         0,         0,    1.0000;
    
//     const double fx = 870.4934000000001;
//     const double fy = 870.4934000000001;
//     const double cx = 689.5979919433594;
//     const double cy = 233.582389831543;
//     const double bf = 445.39666;
//     Eigen::Matrix<double,5,1> Cam;
//     Cam << fx,fy,cx,cy,bf;

//     LidarVisualOptimizer Opti(Tlc,Cam);
//     Opti.AddPlaneEdge = (AddPlaneEdge==1);

//     //Step1 . Read Visual Frame and Mappoint
//     Opti.ReadTxT("/home/qk/Documents/dj2_MapPoint");
//     cout << "ViusalFrame size = " << Opti.VisualFrames.size() << endl;
    
//     //Step2 . Read Lidar Map Pcd
//     Opti.ReadPcd("/home/qk/Documents/dj2_MapPoint/LidarMap.pcd");
//     cout << "Lidar point size = " << Opti.cloud_lidar->points.size() << endl;

//     //Step3 . Read G2OFilse from interactive slam 
//     // 只根据LidarNearFrame的长度读取所需部分
//     Opti.Readg2o("/home/qk/Documents/dj2_MapPoint/graph.g2o");
//     // Opti.Readg2o("/home/qk/Desktop/dj2_withPlane_1/graph.g2o");
//     cout << "SE3 Edge Num = " << Opti.SE3_Info.size() << endl;
    
//     //Step4 . Extract Plane
//     Opti.GetPlanePoint();
//     if(ShowPlane == 1)
//         Opti.ShowPlaneEx();

//     //Step5 . Init Optimizer
//     Opti.InitOptimizer();
//     Opti.AddVE();
//     Opti.Optimize(20);

//     if(ShowReproj == 1) 
//         ReProj2Pic(Opti,FrameId);

//     if(ShowOptiResult == 1)
//         Opti.OptiResultShow();

// }

LidarVisualOptimizer::LidarVisualOptimizer(Eigen::Matrix4d T,Eigen::Matrix<double,5,1> Cam)
{
    Tlc = T;
    Tcl = T.inverse();
    fx = Cam(0);
    fy = Cam(1);
    cx = Cam(2);
    cy = Cam(3);
    bf = Cam(4);
}

void LidarVisualOptimizer::ReadTxT(string Path)
{
    // string path = "/home/qk/Documents/dj2_MapPoint/DEMO_1";

    //get Mappoint
    ifstream ifs(Path+"/GoodMapPoints.txt");
    if (!ifs)
    {
        cout << " open fail GoodMapPoint " << endl;
        return;
    }
    string s;
    MapPoint mp;
    ifs >> s;
    while (!ifs.eof())
    {
        if( s == "Id")
        {
            int id;
            ifs >> id;
            mp.id = id;
        }
        else if( s == "ObsNum")
        {
            int Obs_num;
            ifs >> Obs_num;
            for (int i = 0; i < Obs_num; i++)
            {
                int ii,frame_id,index;
                ifs >> ii;
                ifs >> frame_id;
                ifs >> index;
                Observation ob;
                ob.FeaIndex = index;
                ob.FrameID = frame_id;
                mp.Observations.push_back(ob);
                // cout << "Obs " << ii << " " << frame_id << " " << index << endl;
            }
        }
        else if( s == "WorldPos")
        {
            double x,y,z;
            ifs >> x;
            ifs >> y;
            ifs >> z;
            // cout << "World Pos =  "  << x << " " << y << " " << z << endl;
            mp.WorldPos(0) = x;
            mp.WorldPos(1) = y;
            mp.WorldPos(2) = z;
            VisualMapPoints.push_back(mp);
            VisualMapPoints_raw.push_back(mp);
            mp.Observations.clear();
        }
        ifs >> s;
    }
    ifs.close();

    //Get GoodFrames
    ifstream ifs2(Path+"/GoodFrames.txt");
    if (!ifs2)
    {
        cout << " open fail GoodMapPoint " << endl;
        return;
    }

    Stereo::kFrame f;
    ifs2 >> s;
    while (!ifs2.eof())
    {
        if(s == "Frame")
        {
            int Frame_id;
            double info;
            ifs2 >> Frame_id;
            ifs2 >> info;
            f.id = Frame_id;
            f.info = info;
        }
        else if(s == "Time")
        {
            double time;
            ifs2 >> time;
            f.time = time;
        }
        else if(s == "Pose")
        {
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    double p;
                    ifs2 >> p;
                    f.pose(i,j) = p;
                }   
            }
        }
        else if(s == "FeaNum")
        {
            int feanum;
            ifs2 >> feanum;
            for (int i = 0; i < feanum; i++)
            {
                int ii,fea_id,u,v;
                double d;
                ifs2 >> ii;
                ifs2 >> fea_id;
                ifs2 >> u;
                ifs2 >> v;
                ifs2 >> d;
                FeaPoint fp;
                fp.id = fea_id;
                fp.p.x = u;
                fp.p.y = v;
                fp.dis = d;
                f.FeaPoints.push_back(fp); 
            }

            VisualFrames.push_back(f);
            VisualFrames_raw.push_back(f);
            f.FeaPoints.clear();
            
        }
        ifs2 >> s;
    }
    
    //Get LidarNearFrame
    ifstream ifs3(Path+"/LidarNearFrame.txt");
    if (!ifs3)
    {
        cout << " open fail LidarNearFrame " << endl;
        return;
    }

    int id;
    ifs3 >> id;
    while (!ifs3.eof())
    {
        LidarNearFrame.push_back(id);
        ifs3 >> id;
    }

}

void LidarVisualOptimizer::ReadPcd(string Path)
{
    // 读取激光地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(Path, *cloud) == -1)//*打开点云文件。
    {                                                                           
      PCL_ERROR("Couldn't read that pcd file\n");                             // //不带路径的格式【只是把路径删掉即可】
      return;
    }
    cloud_lidar = cloud;
}

void LidarVisualOptimizer::Readg2o(string Path)
{
    int LidarNum = LidarNearFrame.size();
    ifstream ifs(Path);
    if (!ifs)
    {
        cout << " open fail LidarNearFrame " << endl;
        return;
    }


    string name;
    while (!ifs.eof())
    {
        ifs >> name;
        if(name == "VERTEX_SE3:QUAT")
        {
            int id;
            ifs >> id;
            if(id <= LidarNum)
            {
                Eigen::Matrix<double,7,1> est;
                for (int i=0; i<7; i++)
                    ifs  >> est[i];
                g2o::SE3Quat Twl;
                Twl.fromVector(est);
                // cout << "T " << Twl.to_homogeneous_matrix() << endl;
                kFrame f;
                f.id = id;
                f.pose = Twl.to_homogeneous_matrix();
                LidarFrames.push_back(f);
            }
        }

        if(name == "EDGE_SE3:QUAT")
        {
            int id1,id2;
            ifs >> id1;
            ifs >> id2;
            Eigen::Matrix<double,7,1> est;
            for (int i=0; i<7; i++)
                ifs  >> est[i];
            g2o::SE3Quat Twl;
            Twl.fromVector(est);
            if(id1 <= LidarNum && id2 <= LidarNum)
            {
                EdgeSE3Info e;
                if(id1 > 0)
                    id1 = id1-1;
                if(id2 > 0)
                    id2 = id2-1;
                e.Id1 = id1;
                e.Id2 = id2;
                e.dT = Twl;
                SE3_Info.push_back(e);
            }
        }

        if(name == "VERTEX_PLANE")
        {
            int id;
            ifs >> id;
            double a,b,c,d;
            ifs >> a;
            ifs >> b;
            ifs >> c;
            ifs >> d;
            Eigen::Vector4d Plane_coff(a,b,c,d);
            PlaneAndPoint.Plane_coffs.push_back(Plane_coff);
            PlaneAndPoint.PlaneId.push_back(id);
            PlaneAndPoint.FrameId.resize(PlaneAndPoint.PlaneId.size());
        }

        if(name == "EDGE_SE3_PLANE")
        {
            int id1,id2;
            ifs >> id1;
            ifs >> id2;

            if(id1 <= LidarNum)
            {
                for (int i = 0; i < PlaneAndPoint.PlaneId.size(); i++)
                {
                    if(id2 == PlaneAndPoint.PlaneId[i])
                    {
                        PlaneAndPoint.FrameId[i].push_back(id1);

                        EdgeSE3PlaneInfo e;
                        if(id1 == 0)
                            id1 = 1;
                        e.Id1 = id1 - 1;
                        e.Id2 = i;
                        double A,B,C,D;
                        ifs >> A;
                        ifs >> B;
                        ifs >> C;
                        ifs >> D;
                        e.Plane << A,B,C,D;
                        SE3_Plane_Info.push_back(e);
                    }
                }
                
            }
        }

    }
}

g2o::SE3Quat LidarVisualOptimizer::T2SE3Quat(Eigen::Matrix4d T)
{
    Eigen::Matrix3d R = T.block(0,0,3,3);
    Eigen::Vector3d t = Eigen::Vector3d(T(0,3),T(1,3),T(2,3));
    g2o::SE3Quat SE3 = g2o::SE3Quat(R,t);
    return SE3;
}

void LidarVisualOptimizer::SetTlc(Eigen::Matrix4d T)
{
    Tlc = T;
    Tcl = T.inverse(); 
}

void LidarVisualOptimizer::GetPlanePoint()
{
    // Step .1 　对原始激光地图降采样
    // voxelGrid降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_lidar);
    sor.setLeafSize(0.2f,0.2f,0.2f);
    sor.filter(*cloud_filtered);

    // Step .2 依次对每个平面提取
    for (int i = 0; i < PlaneAndPoint.PlaneId.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
        // Steo . 2.1 提取激光平面
        double A = PlaneAndPoint.Plane_coffs[i][0];
        double B = PlaneAndPoint.Plane_coffs[i][1];
        double C = PlaneAndPoint.Plane_coffs[i][2];
        double D = PlaneAndPoint.Plane_coffs[i][3];
        double plane_dis = 0.1; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
        for (int j = 0; j < cloud_filtered->size(); j++)
        {
            double x,y,z;
            x = cloud_filtered->points[j].x;
            y = cloud_filtered->points[j].y;
            z = cloud_filtered->points[j].z;
            double distance,distance2;
            distance = abs(A*x+B*y+C*z+D);
            // distance2 = abs(A2*x+B2*y+C2*z+D2);
            if(distance < plane_dis)
            {   
                pcl::PointXYZ p;
                p.x = x;
                p.y = y;
                p.z = z;
                cloud_plane->points.push_back(p);
            }
        }
        cloud_plane->height = 1;
        cloud_plane->width = cloud_plane->points.size();

        //Step . 2.2 激光平面简单处理
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ>::Ptr out_sor(new pcl::StatisticalOutlierRemoval<pcl::PointXYZ>());
        out_sor->setMeanK(50);
        out_sor->setStddevMulThresh(1.0);

        out_sor->setInputCloud(cloud_plane);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_out(new pcl::PointCloud<pcl::PointXYZ>);
        out_sor->filter(*cloud_plane_out);

        for (int i = 0; i < cloud_plane_out->size(); i++)
        {
            pcl::PointXYZ p1 = cloud_plane_out->points[i];
            int NearNum = 0;
            for (int j = 0; j  < cloud_plane_out->size(); j++)
            {
                pcl::PointXYZ p2 = cloud_plane_out->points[j];

                int dx = abs(p1.x - p2.x);
                int dy = abs(p1.y - p2.y);
                int dz = abs(p1.z - p2.z);
                if(dx < 0.2)
                    if(dy < 0.2 && dz < 0.2)
                        NearNum++;
            }
            // cout << "Near Num = " << NearNum << endl;   
            if(NearNum > 40)
                cloud_final->points.push_back(p1);
        }

        //Step . 2.3 提取视觉平面
        double disTH = 1.0;
        vector<Stereo::MapPoint> GoodMapPoints_NearPlane_raw;
        vector<Stereo::MapPoint> GoodMapPoints_NearPlane;
        for (int i = 0; i <VisualMapPoints.size(); i++)
        {
            double x = VisualMapPoints[i].WorldPos[0];
            double y = VisualMapPoints[i].WorldPos[1];
            double z = VisualMapPoints[i].WorldPos[2];
            double distance = abs(A*x+B*y+C*z+D);
            if(distance < disTH)
                GoodMapPoints_NearPlane_raw.push_back(VisualMapPoints[i]);
        }   

        
        // 找到lidar最近邻　判断是否outlier
        for (int i = 0; i < GoodMapPoints_NearPlane_raw.size(); i++)
        {
            double min_dis = 10000.0;

            Eigen::Vector3d w(1,1,1) ;
            if(abs(C) < 0.1)
                w[2] = 5;

            for (int j = 0; j < cloud_final->points.size(); j++)
            {
                double dis = w[0]*abs(cloud_final->points[j].x -  GoodMapPoints_NearPlane_raw[i].WorldPos[0]) + w[1]*abs(cloud_final->points[j].y -  GoodMapPoints_NearPlane_raw[i].WorldPos[1]) + w[2]*abs(cloud_final->points[j].z -  GoodMapPoints_NearPlane_raw[i].WorldPos[2]);
                if(dis < min_dis)
                {
                    min_dis = dis;
                }
            }
            if(min_dis < disTH)
                GoodMapPoints_NearPlane.push_back(GoodMapPoints_NearPlane_raw[i]);
        }

        PlaneAndPoint.MapPoints.push_back(GoodMapPoints_NearPlane);
        clouds_plane.push_back(cloud_final);
    }

}

void LidarVisualOptimizer::ShowPlaneEx()
{
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

        // 画出近平面点
        glPointSize(3);
        glBegin(GL_POINTS);
        for (int i = 0; i < PlaneAndPoint.Plane_coffs.size(); i++)
        {
            for (int j = 0; j < PlaneAndPoint.MapPoints[i].size(); j++)
            {
                glColor3f(1.0,0.0,0.0);
                
                glVertex3d(PlaneAndPoint.MapPoints[i][j].WorldPos(0),PlaneAndPoint.MapPoints[i][j].WorldPos(1),PlaneAndPoint.MapPoints[i][j].WorldPos(2));
            }
            
        }
        glEnd();

        // 激光近平面点
        glPointSize(2);
        glBegin(GL_POINTS);
        for (int i = 0; i < clouds_plane.size(); i++)
        {
            for (int j = 0; j < clouds_plane[i]->points.size(); j++)
            {
                glColor3f(0.6, 0.6, 0.6);
                glVertex3d(clouds_plane[i]->points[j].x,clouds_plane[i]->points[j].y,clouds_plane[i]->points[j].z);
            }
            
        }
        glEnd();

        pangolin::FinishFrame();
        // pangolin::DestroyWindow("Trajectory Viewer");
    }
}

void LidarVisualOptimizer::InitOptimizer()
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1,-1>> Block;
    unique_ptr<Block::LinearSolverType> linearSolver( new g2o::LinearSolverEigen<Block::PoseMatrixType>() );
    unique_ptr<Block> solver_ptr (new Block (std::move(linearSolver)));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));
    // g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    thHuber3D = sqrt(7.815);
    cout << "***********************" << endl;
    cout << "Optimizer Init ! " << endl;
}

void LidarVisualOptimizer::AddVE()
{
    // 1. 加入外参顶点 Tlc

    g2o::VertexSE3Expmap *vTcl = new g2o::VertexSE3Expmap();
    g2o::SE3Quat T = T2SE3Quat(Tcl);
    vTcl->setEstimate(T);
    vTcl->setId(0);
    // vTcl->setFixed(true);
    optimizer.addVertex(vTcl);
    VertexNum ++;
    cout << "  *Tcl VertexNum = " << VertexNum << endl;

    IdVisual = VertexNum;
    // 2. 加入视觉顶点
    for (int i = 0; i < VisualFrames.size(); i++)
    {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setEstimate(T2SE3Quat(VisualFrames[i].pose.inverse()));
        v->setId(VertexNum+VisualFrames[i].id);
        // if(i == 0 )
        //     v->setFixed(true);
        // if(i == VisualFrames.size()-1 )
        //     v->setFixed(true);
        optimizer.addVertex(v);
    }
    VertexNum += VisualFrames.size();
    cout << "  *Visual VertexNum = " << VisualFrames.size() << endl;

    IdLidar = VertexNum;
    // 3. 加入激光顶点
    for (int i = 0; i < LidarFrames.size()-1; i++)
    {
        // cout << " i = " << i << endl;
        // cout << "Lidar Pose = " << endl << LidarFrames[i+1].pose << endl;
        // int inear = LidarNearFrame[i];
        // cout << "Visual Near pose = " << endl << VisualFrames[inear].pose << endl;

        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setEstimate(T2SE3Quat(LidarFrames[i+1].pose.inverse()));
        v->setId(VertexNum + i);
        v->setFixed(true);
        optimizer.addVertex(v);
    }
    cout << "  *Lidar VertexNum = " << LidarFrames.size()-1 << endl;
    VertexNum += LidarFrames.size()-1;

    IdPoint = VertexNum;
    // 4. 加入三维点
    for (int i = 0; i < VisualMapPoints.size(); i++)
    {
        g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
        v->setEstimate(VisualMapPoints[i].WorldPos);
        v->setId(VertexNum + i);
        v->setMarginalized(true);
        optimizer.addVertex(v);

    }
    VertexNum += VisualMapPoints.size();
    cout << "  *Point VertexNum = " << VisualMapPoints.size() << endl;

    IdPlane = VertexNum;
    // 5. 加入平面
    for (int i = 0; i < PlaneAndPoint.PlaneId.size(); i++)
    {
        VertexPlane *v = new VertexPlane();
        v->setEstimate(PlaneAndPoint.Plane_coffs[i]);
        v->setId(VertexNum + i);
        // cout << "Plane Id :" << VertexNum+i << endl;
        v->setFixed(true);
        optimizer.addVertex(v);
    }
    VertexNum += PlaneAndPoint.Plane_coffs.size();
    cout << "  *Plane VertexNum = " << PlaneAndPoint.Plane_coffs.size() << endl;

    cout << "All Vertex Num = " << VertexNum << endl;

    // 6. 加入小平面
    IdNormPlane = VertexNum;
    for (int i = 0; i < PlaneAndPoint_Norm.Plane_coffs.size(); i++)
    {
        VertexPlane *v = new VertexPlane();
        v->setEstimate(PlaneAndPoint_Norm.Plane_coffs[i]);
        v->setId(VertexNum+i);
        v->setFixed(true);
        optimizer.addVertex(v);
    }
    
    // 构造边

    // 1. 三元边 Tcl Tvisual Point

    for (int i = 0; i < VisualMapPoints.size(); i++)
    {
        int obs_num = VisualMapPoints[i].Observations.size();
        for (int j = 0; j < obs_num; j++)
        {
            Eigen::Vector3d obs;
            int Feaid = VisualMapPoints[i].Observations[j].FeaIndex;
            int Frameid = VisualMapPoints[i].Observations[j].FrameID;
            obs(0) = (double)VisualFrames[Frameid].FeaPoints[Feaid].p.x;
            obs(1) = (double)VisualFrames[Frameid].FeaPoints[Feaid].p.y;
            obs(2) = VisualFrames[Frameid].FeaPoints[Feaid].dis;

            My3Edge *e = new My3Edge();
            e->resize(3);
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdPoint+i)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
            e->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdVisual+Frameid)));
            Eigen::Matrix3d Info = Eigen::Matrix3d::Identity();
            e->setMeasurement(obs);
            e->setInformation(Info);
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(thHuber3D);
            e->fx = fx;
            e->fy = fy;
            e->cx = cx;
            e->cy = cy;
            e->bf = bf;
            optimizer.addEdge(e);
        }
        
    }
    
    // 2. 二元边　Tviusal Tlidar
    // cout << "LidarNear size = " <<LidarNearFrame.size() << endl;
    // for (int i = 0; i < LidarNearFrame.size(); i++)
    // {
    //     int frame_v = LidarNearFrame[i];
    //     int frame_l = i;
    //     g2o::EdgeSE3Expmap *e = new g2o::EdgeSE3Expmap();
    //     e->setMeasurement(T2SE3Quat(Eigen::Matrix4d::Identity()));
    //     e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdVisual+frame_v)));
    //     e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdLidar+frame_l)));
    //     Eigen::Matrix<double,6,6> Info = Eigen::Matrix<double,6,6>::Identity();
    //     Info = Info*10000.0;
    //     Info(3,3) = 40000.0;
    //     Info(4,4) = 40000.0;
    //     Info(5,5) = 40000.0;
    //     e->setInformation(Info);
    //     optimizer.addEdge(e);
    // }
    

    // 3. 二元边　Plane Point
    if(AddPlaneEdge == 1 ||AddPlaneEdge == 3)
    {
        for (int p = 0; p < PlaneAndPoint.PlaneId.size(); p++)
        {
            for (int i = 0; i < PlaneAndPoint.MapPoints[p].size(); i++)
            {
                int Pi = PlaneAndPoint.MapPoints[p][i].id;
                EdgePointPlane *e = new EdgePointPlane();
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdPoint+Pi)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdPlane+p)));
                Eigen::Matrix<double,1,1> Info = Eigen::Matrix<double,1,1>::Identity();
                e->setMeasurement(0.0);
                e->setInformation(10*Info);
                optimizer.addEdge(e);
            }
        }
    }

    if(AddPlaneEdge == 2 || AddPlaneEdge == 3)
    {
        for (int i = 0; i < PlaneAndPoint_Norm.Plane_coffs.size(); i++)
        {
            int Pi = PlaneAndPoint_Norm.MapPoint[i].id;
            EdgePointPlane *e = new EdgePointPlane();
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdPoint+Pi)));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdNormPlane+i)));
            Eigen::Matrix<double,1,1> Info = Eigen::Matrix<double,1,1>::Identity();
            e->setMeasurement(0.0);
            e->setInformation(5*Info);
            optimizer.addEdge(e);
        }
        
    }

    // 4. 二元边　Tlidar Tlidar

    // for (int i = 0; i < SE3_Info.size(); i++)
    // {

    //     // cout << "********************************************************************" << endl;
    //     // cout << "id " << SE3_Info[i].Id1 << " " << SE3_Info[i].Id2 << " dT " << endl << SE3_Info[i].dT << endl; 

    //     int id1 = LidarNearFrame[SE3_Info[i].Id1];
    //     int id2 = LidarNearFrame[SE3_Info[i].Id2];

    //     Eigen::Matrix4d T1 = VisualFrames[id1].pose;
    //     Eigen::Matrix4d T1_lidar = LidarFrames[SE3_Info[i].Id1+1].pose;
    //     Eigen::Matrix4d T2 = VisualFrames[id2].pose;
    //     Eigen::Matrix4d T2_lidar = LidarFrames[SE3_Info[i].Id2+1].pose;

    //     Eigen::Matrix4d dT = SE3_Info[i].dT.to_homogeneous_matrix();
    //     dT = T1.inverse()*T1_lidar*dT*T2.inverse()*T2_lidar;
    //     // cout << "T1*dT lidar" << endl <<  T1*dT << endl;
    //     // cout << "T2 lidar  " << endl << T2 << endl;
    //     g2o::EdgeSE3Expmap *e = new g2o::EdgeSE3Expmap();
    //     e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdVisual+id2)));
    //     e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(IdVisual+id1)));
    //     e->setMeasurement(T2SE3Quat(dT));
    //     Eigen::Matrix<double,6,6> Info = Eigen::Matrix<double,6,6>::Identity();
    //     Info = 1000*Info;
    //     Info(3,3) = 4000.0;
    //     Info(4,4) = 4000.0;
    //     Info(5,5) = 4000.0;
    //     e->setInformation(Info);
    //     optimizer.addEdge(e);
    // }
    

    // 5. 二元边　Tlidar Plane

    // for (int i = 0; i <SE3_Plane_Info.size(); i++)
    // {
    //     cout << " id 1 " << SE3_Plane_Info[i].Id1 << "  id2 = " << SE3_Plane_Info[i].Id2 << " Coffs = " << SE3_Plane_Info[i].Plane << endl;
    //     Eigen::Matrix4d T1 = VisualFrames[SE3_Plane_Info[i].Id1].pose;

    //     Eigen::Vector4d P;
    //     P << 0,0,-SE3_Plane_Info[i].Plane(3)/SE3_Plane_Info[i].Plane(2),1;
    //     Eigen::Vector4d Pw = T1*P;
    //     Eigen::Matrix3d R = T1.block(0,0,3,3);
    //     Eigen::Vector3d n(SE3_Plane_Info[i].Plane(0),SE3_Plane_Info[i].Plane(1),SE3_Plane_Info[i].Plane(2));
    //     Eigen::Vector3d nr = R*n;
    //     double D = nr(0)*Pw(0) + nr(2)*Pw(2) +nr(1)*Pw(1);
    //     cout << " T palne " << endl << nr << " D = " << D << endl;
    // }
    
    cout << "***********************" << endl;

}

void LidarVisualOptimizer::Optimize(int Num)
{
    optimizer.initializeOptimization();
    optimizer.optimize(Num);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < VisualMapPoints.size(); i++)
    {
        g2o::VertexSBAPointXYZ* v = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(IdPoint+i));
        VisualMapPoints[i].WorldPos = v->estimate();
        pcl::PointXYZ p;
        p.x = v->estimate()(0);
        p.y = v->estimate()(1);
        p.z = v->estimate()(2);
        cloud->points.push_back(p);
    }
    cloud_visual = cloud;
    // 2.Get Frames
    for (int i = 0; i < VisualFrames.size(); i++)
    {
        g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(IdVisual+i));
        Eigen::Matrix4d T = v->estimate().to_homogeneous_matrix();
        VisualFrames[i].pose = T.inverse();
    }

    // 2.1 Get Tbc
    g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    // g2o::VertexSE3 *v = static_cast<g2o::VertexSE3*>(optimizer.vertex(0));
    // g2o::SE3Quat SE3T = v->estimate();
    Eigen::Matrix4d Tcl_a = v->estimate().to_homogeneous_matrix();
    Eigen::Matrix4d Tlc_a = Tcl_a.inverse();
    cout << "Tlc before = " << endl << Tlc << endl;
    cout << "Tlc after = " << endl << Tlc_a << endl;
    Tlc_o = Tlc_a;
    
}

void LidarVisualOptimizer::OptiResultShow()
{

    // 3.转换轨迹
    Poses pose_after = Frame2Isom(VisualFrames);
    Poses pose_raw = Frame2Isom(VisualFrames_raw);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_lidar);
    sor.setLeafSize(0.2f,0.2f,0.2f);
    sor.filter(*cloud_filtered);


    // 4.可视化
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

        // 优化轨迹
        for (size_t ip = 0; ip < pose_after.size()-2; ip++) {
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = pose_after[ip], p2 = pose_after[ip + 1];
            glVertex3d( 1 * p1.translation()[0], 1 * p1.translation()[1], 1 * p1.translation()[2]);
            glVertex3d( 1 * p2.translation()[0], 1 * p2.translation()[1], 1 * p2.translation()[2]);
            glEnd();
        }

        // 原始轨迹
        for (size_t ip = 0; ip < pose_raw.size()-2; ip++) {
            glColor3f(0.5, 0.5, 0.5);
            glBegin(GL_LINES);
            auto p1 = pose_raw[ip], p2 = pose_raw[ip + 1];
            glVertex3d( 1 * p1.translation()[0], 1 * p1.translation()[1], 1 * p1.translation()[2]);
            glVertex3d( 1 * p2.translation()[0], 1 * p2.translation()[1], 1 * p2.translation()[2]);
            glEnd();
        }

        // 画出激光地图点
        glPointSize(2);
        glBegin(GL_POINTS);

        for (int i = 0; i < clouds_plane.size(); i++)
        {
            for (int j = 0; j < clouds_plane[i]->points.size(); j++)
            {
                glColor3f(0, 0.6, 0.6);
                glVertex3d(clouds_plane[i]->points[j].x,clouds_plane[i]->points[j].y,clouds_plane[i]->points[j].z);
            }
        }
        glEnd();

        // 画出激光地图点
        glPointSize(1);
        glBegin(GL_POINTS);

        for (int j = 0; j < cloud_filtered->points.size(); j++)
        {
            glColor3f(0, 0, 0);
            glVertex3d(cloud_filtered->points[j].x,cloud_filtered->points[j].y,cloud_filtered->points[j].z);
        }
        
        glEnd();

        // 画出地图点
        glPointSize(3);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < VisualMapPoints.size(); i++)
        {
            glColor3f(1.0, 0.0, 0.0);
            glVertex3d(VisualMapPoints[i].WorldPos(0),VisualMapPoints[i].WorldPos(1),VisualMapPoints[i].WorldPos(2));
        }
        glEnd();

        pangolin::FinishFrame();
    }
    
    
}

vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> LidarVisualOptimizer::Frame2Isom(vector<Stereo::kFrame> Frames)
{
    vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    for (int i = 0; i < Frames.size(); i++)
    {
        Eigen::Matrix4d T_f = Frames[i].pose*Tlc;
        Eigen::Matrix3d R_f;
        R_f = T_f.block(0,0,3,3);
        Eigen::Quaterniond Q(R_f);
        Eigen::Isometry3d Twr(Eigen::Quaterniond(Q.w(),Q.x(),Q.y(),Q.z()));
        Twr.pretranslate(Eigen::Vector3d(T_f(0,3),T_f(1,3),T_f(2,3)));
        poses.push_back(Twr);
    }    
    return poses;
}

void LidarVisualOptimizer::GetPlaneNorm()
{
    // Step 1. 降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_lidar);
    sor.setLeafSize(0.2f,0.2f,0.2f);
    sor.filter(*cloud_filtered);

    // Step 2.　计算法向量
    pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals_filtered(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::search::Octree<pcl::PointXYZ>::Ptr tree(new pcl::search::Octree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);
    n.setInputCloud(cloud_filtered);
    n.setSearchMethod(tree);
    n.setKSearch(NearPointNum);
    // n.setRadiusSearch(0.5);
    n.compute(*normals);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_norm(new pcl::PointCloud<pcl::PointXYZ>);
    // Step 3. 根据曲率得到较好平面
    for (int i = 0; i < normals->points.size(); i++)
    {
        if(abs(normals->points[i].curvature) < 0.005)
        {
            cloud_norm->points.push_back(cloud_filtered->points[i]);
            normals_filtered->points.push_back(normals->points[i]);
        }
    }
    cloud_norm_ = cloud_norm;
    
    // Step 4. 遍历视觉点寻找近邻

    for (int j = 0; j < VisualMapPoints.size(); j++)
    {
        double min_dis = 10000;
        int min_id = 0;
        for(int i = 0; i < cloud_norm->points.size(); i++)
        {
            double vx = cloud_norm->points[i].x - VisualMapPoints[j].WorldPos(0);
            double vy = cloud_norm->points[i].y - VisualMapPoints[j].WorldPos(1);
            double vz = cloud_norm->points[i].z - VisualMapPoints[j].WorldPos(2);

            // dis = abs(normals_filtered->points[i].normal_x*vx+normals_filtered->points[i].normal_y*vy+normals_filtered->points[i].normal_z*vz);
            double dis = abs(vx) + abs(vy) + abs(vz);  
            if(dis < min_dis)
            {
                min_dis = dis;
                min_id = i;
            }
        }
        if(min_dis < 1.2)
        {
            double vx = cloud_norm->points[min_id].x - VisualMapPoints[j].WorldPos(0);
            double vy = cloud_norm->points[min_id].y - VisualMapPoints[j].WorldPos(1);
            double vz = cloud_norm->points[min_id].z - VisualMapPoints[j].WorldPos(2);
            double dis_plane = abs(normals_filtered->points[min_id].normal_x*vx+normals_filtered->points[min_id].normal_y*vy+normals_filtered->points[min_id].normal_z*vz);
            if(dis_plane < 0.8)
            {
                PlaneAndPoint_Norm.MapPoint.push_back(VisualMapPoints[j]);
                //计算平面系数
                Eigen::Vector4d Plane_co;
                Plane_co(0) = normals_filtered->points[min_id].normal_x;
                Plane_co(1) = normals_filtered->points[min_id].normal_y;
                Plane_co(2) = normals_filtered->points[min_id].normal_z;
                Plane_co(3) = -(Plane_co(0)*cloud_norm->points[min_id].x+Plane_co(1)*cloud_norm->points[min_id].y+Plane_co(2)*cloud_norm->points[min_id].z);
                PlaneAndPoint_Norm.Plane_coffs.push_back(Plane_co);
            }

        }
    }



}


void ReProj2Pic(LidarVisualOptimizer &Opti,int FrameId)
{
    //冲投影回图片
    string time = to_string((int64)(Opti.VisualFrames[FrameId].time*1000000000));
    cv::Mat Image = cv::imread("/home/qk/Documents/dj2_MapPoint/dj2_stereo_image/Left/" + time + ".jpg");
    g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(Opti.optimizer.vertex(Opti.IdVisual+FrameId));
    Eigen::Matrix4d Tlw = v->estimate().to_homogeneous_matrix();
    Eigen::Matrix4d Tcl= Opti.Tlc_o.inverse();
    cout << "visual points size = " << Opti.cloud_visual->points.size() << endl;

    Eigen::Matrix4d Q;
    Q << 1,0,0,-Opti.cx,
        0,1,0,-Opti.cy,
        0,0,0,Opti.fx,
        0,0,0.001*Opti.fx/Opti.bf,0;
    Eigen::Matrix4d Q_inv = Q.inverse();
    cv::Mat im = Image.clone();
    cout << "Image row = " << Image.rows << endl;
    cout << "Image col = " << Image.cols << endl;
    cv::Mat Depth(Image.rows,Image.cols,CV_16UC1);
    cv::Mat Lidar_Depth(Image.rows,Image.cols,CV_16UC1);

    // 重投影激光点
    // voxelGrid降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(Opti.cloud_lidar);
    sor.setLeafSize(0.4f,0.4f,0.4f);
    sor.filter(*cloud_filtered);


    for (int i = 0; i < cloud_filtered->points.size() ; i++)
    {
        Eigen::Vector4d Pc,Pw;
        Pw[0] = cloud_filtered->points[i].x;
        Pw[1] = cloud_filtered->points[i].y;
        Pw[2] = cloud_filtered->points[i].z;
        Pw[3] = 1;
        
        Pc = Tcl*Tlw*Pw;

        double s = Q_inv(3,2)*Pc(2);
        int u = (Pc(0)+Pc(2)*Q_inv(0,2))/s;
        if(u <= 0 || u >= 1280)
            continue;
        int v = (Pc(1)+Pc(2)*Q_inv(1,2))/s;
        if(v <= 0 || v >= 512)
            continue;
        double depth = Pc[2] * 1000.0;

        if(depth <0.5 || depth > 40000)
            Lidar_Depth.at<unsigned short>(v,u) = 0;
        else
        {
            cv::Point2i p;
            p.x = u;
            p.y = v;
            unsigned short d = depth;
            Lidar_Depth.at<unsigned short>(v,u) = d;
            int rgb = (int)(255.0*depth/65535);
            rgb = rgb*2;

            cv::circle(im,p,1,CV_RGB(0,rgb,0),-1);
        }
    }

    // for (int i = 0; i < Opti.cloud_norm_->points.size() ; i++)
    // {
    //     Eigen::Vector4d Pc,Pw;
    //     Pw[0] = Opti.cloud_norm_->points[i].x;
    //     Pw[1] = Opti.cloud_norm_->points[i].y;
    //     Pw[2] = Opti.cloud_norm_->points[i].z;
    //     Pw[3] = 1;
        
    //     Pc = Tcl*Tlw*Pw;

    //     double s = Q_inv(3,2)*Pc(2);
    //     int u = (Pc(0)+Pc(2)*Q_inv(0,2))/s;
    //     if(u <= 0 || u >= 1280)
    //         continue;
    //     int v = (Pc(1)+Pc(2)*Q_inv(1,2))/s;
    //     if(v <= 0 || v >= 512)
    //         continue;
    //     double depth = Pc[2] * 1000.0;

    //     if(depth <0.5 || depth > 40000)
    //         Lidar_Depth.at<unsigned short>(v,u) = 0;
    //     else
    //     {
    //         cv::Point2i p;
    //         p.x = u;
    //         p.y = v;
    //         unsigned short d = depth;
    //         Lidar_Depth.at<unsigned short>(v,u) = d;
    //         int rgb = (int)(255.0*depth/65535);
    //         rgb = rgb*2;

    //         cv::circle(im,p,1,CV_RGB(0,rgb,0),-1);
    //     }
    // }

    // 重投影视觉点
    for (int i = 0; i < Opti.cloud_visual->points.size() ; i++)
    {
        Eigen::Vector4d Pc,Pw;
        Pw[0] = Opti.cloud_visual->points[i].x;
        Pw[1] = Opti.cloud_visual->points[i].y;
        Pw[2] = Opti.cloud_visual->points[i].z;
        Pw[3] = 1;
        
        Pc = Tcl*Tlw*Pw;

        double s = Q_inv(3,2)*Pc(2);
        int u = (Pc(0)+Pc(2)*Q_inv(0,2))/s;
        if(u <= 0 || u >= 1280)
            continue;
        int v = (Pc(1)+Pc(2)*Q_inv(1,2))/s;
        if(v <= 0 || v >= 512)
            continue;
        double depth = Pc[2] * 1000.0;

        if(depth <0.5 || depth > 40000)
            Lidar_Depth.at<unsigned short>(v,u) = 0;
        else
        {
            cv::Point2i p;
            p.x = u;
            p.y = v;
            unsigned short d = depth;
            Lidar_Depth.at<unsigned short>(v,u) = d;
            int rgb = (int)(255.0*depth/65535);
            rgb = rgb*2;

            cv::circle(im,p,1,CV_RGB(rgb,0,0),-1);
        }
    }

    cv::imshow("im",im);
    cv::waitKey();
}





MyEdgeSE3::MyEdgeSE3() :
  g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>() {
}

void MyEdgeSE3::linearizeOplus() {
  g2o::VertexSE3Expmap * vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
  g2o::SE3Quat Ti(vi->estimate());

  g2o::VertexSE3Expmap * vj = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
  g2o::SE3Quat Tj(vj->estimate().inverse());

  const g2o::SE3Quat & Tij = _measurement;
  g2o::SE3Quat invTij = Tij.inverse();

  g2o::SE3Quat invTj_Tij = Tj.inverse()*Tij;
  g2o::SE3Quat infTi_invTij = Ti.inverse()*invTij;

  _jacobianOplusXi = invTj_Tij.adj();
  _jacobianOplusXj = -infTi_invTij.adj();
}

bool MyEdgeSE3::read(std::istream& is)  {
  Eigen::Matrix<double,7,1,0> meas;
  for (int i=0; i<7; i++)
    is >> meas[i];
  g2o::SE3Quat cam2world;
  cam2world.fromVector(meas);
  setMeasurement(cam2world.inverse());
  //TODO: Convert information matrix!!
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++) {
      is >> information()(i,j);
      if (i!=j)
        information()(j,i)=information()(i,j);
    }
  return true;
}

bool MyEdgeSE3::write(std::ostream& os) const {
  g2o::SE3Quat cam2world(measurement().inverse());
  for (int i=0; i<7; i++)
    os << cam2world[i] << " ";
  for (int i=0; i<6; i++)
    for (int j=i; j<6; j++){
      os << " " <<  information()(i,j);
    }
  return os.good();
}



void EdgePointPlane::linearizeOplus()
{
    const VertexPlane *v1 = static_cast<const VertexPlane *>(_vertices[1]);
    Vector4D Plane = v1->estimate();

    _jacobianOplusXi(0,0) = Plane(0);
    _jacobianOplusXi(0,1) = Plane(1);
    _jacobianOplusXi(0,2) = Plane(2);
}


Vector3D My3Edge::cam_project(const Vector3D & trans_xyz, const float &bf) const{
  const float invz = 1.0f/trans_xyz[2];
  Vector3D res;
  res[0] = trans_xyz[0]*invz*fx + cx;
  res[1] = trans_xyz[1]*invz*fy + cy;
  res[2] = res[0] - bf*invz;
  return res;
}

void My3Edge::linearizeOplus(){
    // cout << "linear! "<< endl;
    g2o::VertexSE3Expmap *v0  = static_cast<g2o::VertexSE3Expmap *>(_vertices[1]);
    g2o::VertexSE3Expmap *v1  = static_cast<g2o::VertexSE3Expmap *>(_vertices[2]);
    g2o::VertexSBAPointXYZ* v2 = static_cast<g2o::VertexSBAPointXYZ*>(_vertices[0]);

    g2o::SE3Quat TCL(v0->estimate());
    g2o::SE3Quat TLW(v1->estimate());
    Vector3D Pw = v2->estimate();
    Vector3D Pl = TLW.map(Pw);
    Vector3D Pc = TCL.map(Pl);

    const Eigen::Matrix3d RCL = TCL.rotation().toRotationMatrix();
    const Eigen::Matrix3d RLW = TLW.rotation().toRotationMatrix();
    const Eigen::Matrix3d R = RCL*RLW;

    double x = Pc[0];
    double y = Pc[1];
    double z = Pc[2];
    double z_2  = z*z;

    double xl = Pl[0];
    double yl = Pl[1];
    double zl = Pl[2];

    _jacobianOplus[0](0,0) = -fx*R(0,0)/z+fx*x*R(2,0)/z_2;
    _jacobianOplus[0](0,1) = -fx*R(0,1)/z+fx*x*R(2,1)/z_2;
    _jacobianOplus[0](0,2) = -fx*R(0,2)/z+fx*x*R(2,2)/z_2;

    _jacobianOplus[0](1,0) = -fy*R(1,0)/z+fy*y*R(2,0)/z_2;
    _jacobianOplus[0](1,1) = -fy*R(1,1)/z+fy*y*R(2,1)/z_2;
    _jacobianOplus[0](1,2) = -fy*R(1,2)/z+fy*y*R(2,2)/z_2;

    _jacobianOplus[0](2,0) = _jacobianOplus[0](0,0)-bf*R(2,0)/z_2;
    _jacobianOplus[0](2,1) = _jacobianOplus[0](0,1)-bf*R(2,1)/z_2;
    _jacobianOplus[0](2,2) = _jacobianOplus[0](0,2)-bf*R(2,2)/z_2;

    _jacobianOplus[1](0,0) =  x*y/z_2 *fx;
    _jacobianOplus[1](0,1) = -(1+(x*x/z_2)) *fx;
    _jacobianOplus[1](0,2) = y/z *fx;
    _jacobianOplus[1](0,3) = -1./z *fx;
    _jacobianOplus[1](0,4) = 0;
    _jacobianOplus[1](0,5) = x/z_2 *fx;

    _jacobianOplus[1](1,0) = (1+y*y/z_2) *fy;
    _jacobianOplus[1](1,1) = -x*y/z_2 *fy;
    _jacobianOplus[1](1,2) = -x/z *fy;
    _jacobianOplus[1](1,3) = 0;
    _jacobianOplus[1](1,4) = -1./z *fy;
    _jacobianOplus[1](1,5) = y/z_2 *fy;

    _jacobianOplus[1](2,0) = _jacobianOplus[1](0,0)-bf*y/z_2;
    _jacobianOplus[1](2,1) = _jacobianOplus[1](0,1)+bf*x/z_2;
    _jacobianOplus[1](2,2) = _jacobianOplus[1](0,2);
    _jacobianOplus[1](2,3) = _jacobianOplus[1](0,3);
    _jacobianOplus[1](2,4) = 0;
    _jacobianOplus[1](2,5) = _jacobianOplus[1](0,5)-bf/z_2;

    Eigen::Matrix<double,3,6> PLhat;
    PLhat << 1,0,0,0,zl,-yl,
            0,1,0,-zl,0,xl,
            0,0,1,yl,-xl,0;
    Eigen::Matrix<double,3,6> RLWhat;
    RLWhat = RCL*PLhat;

    _jacobianOplus[2](0,0) = -fx*RLWhat(0,3)/z+fx*x*RLWhat(2,3)/z_2;
    _jacobianOplus[2](0,1) = -fx*RLWhat(0,4)/z+fx*x*RLWhat(2,4)/z_2;
    _jacobianOplus[2](0,2) = -fx*RLWhat(0,5)/z+fx*x*RLWhat(2,5)/z_2;
    _jacobianOplus[2](0,3) = -fx*RLWhat(0,0)/z+fx*x*RLWhat(2,0)/z_2;
    _jacobianOplus[2](0,4) = -fx*RLWhat(0,1)/z+fx*x*RLWhat(2,1)/z_2;
    _jacobianOplus[2](0,5) = -fx*RLWhat(0,2)/z+fx*x*RLWhat(2,2)/z_2;

    _jacobianOplus[2](1,0) = -fy*RLWhat(1,3)/z+fy*y*RLWhat(2,3)/z_2;
    _jacobianOplus[2](1,1) = -fy*RLWhat(1,4)/z+fy*y*RLWhat(2,4)/z_2;
    _jacobianOplus[2](1,2) = -fy*RLWhat(1,5)/z+fy*y*RLWhat(2,5)/z_2;
    _jacobianOplus[2](1,3) = -fy*RLWhat(1,0)/z+fy*y*RLWhat(2,0)/z_2;
    _jacobianOplus[2](1,4) = -fy*RLWhat(1,1)/z+fy*y*RLWhat(2,1)/z_2;
    _jacobianOplus[2](1,5) = -fy*RLWhat(1,2)/z+fy*y*RLWhat(2,2)/z_2;

    _jacobianOplus[2](2,0) = _jacobianOplus[2](0,0)-bf*RLWhat(2,3)/z_2;
    _jacobianOplus[2](2,1) = _jacobianOplus[2](0,1)-bf*RLWhat(2,4)/z_2;
    _jacobianOplus[2](2,2) = _jacobianOplus[2](0,2)-bf*RLWhat(2,5)/z_2;
    _jacobianOplus[2](2,3) = _jacobianOplus[2](0,3)-bf*RLWhat(2,0)/z_2;
    _jacobianOplus[2](2,4) = _jacobianOplus[2](0,4)-bf*RLWhat(2,1)/z_2;
    _jacobianOplus[2](2,5) = _jacobianOplus[2](0,5)-bf*RLWhat(2,2)/z_2;

}
