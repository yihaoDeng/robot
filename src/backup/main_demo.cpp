#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <time.h>
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <cmath>
#include <mutex>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <omp.h>

//////////////////////////////////////robot
#include <netinet/in.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#define PORT 8089
#define alpha 0.000001//定义实数之间的差值
#define PI 3.1415926
//////////////////////////////robot

// A bit of shorthand
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

#define PCD_MODEL_NUM 1

PointCloud::Ptr model_array[PCD_MODEL_NUM];
SurfaceNormals::Ptr model_normals_array[PCD_MODEL_NUM];
LocalFeatures::Ptr model_descriptors_array[PCD_MODEL_NUM];
PointCloud::Ptr grab_point_array[PCD_MODEL_NUM];
PointCloud::Ptr grab_point_normal_array[PCD_MODEL_NUM];

std::mutex mutex;
int position_read_fd, position_write_fd;
int ack_read_fd, ack_write_fd;

//initial for robot

void transform_1(std::string sf, char ca[])
{
    for(int i=0;i<sf.length();i++)
    {
        ca[i]=sf[i];
    }
}

std::string transform_2(double *a)
{
    std::string out;
    for (int i = 0; i < 7; i++)
    {

        out += std::to_string(a[i]);
        out += ',';
    }
    out +="\r";
    return out;
}

double normal(double *v,int sign)//向量归一化
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if(1==sign)
    {
        if(fabs(result-0)>alpha)
        {
            v[0] = v[0]/result;
            v[1] = v[1]/result;
            v[2] = v[2]/result;
        }
    }
    return result;
}

void compute(double *q,double *vectorBefore,double *vectorAfter)//求两个向量变换的四元数
{
    double rotationAxis[3];//旋转轴
    double rotationAngle;//向量夹角
    double dot_sum;
    rotationAxis[0] = vectorBefore[1] * vectorAfter[2] - vectorBefore[2] * vectorAfter[1];
    rotationAxis[1] = vectorBefore[2] * vectorAfter[0] - vectorBefore[0] * vectorAfter[2];
    rotationAxis[2] = vectorBefore[0] * vectorAfter[1] - vectorBefore[1] * vectorAfter[0];
    normal(rotationAxis,1);
    dot_sum = vectorBefore[0] * vectorAfter[0] + vectorBefore[1] * vectorAfter[1] + vectorBefore[2] * vectorAfter[2];
    double angle = dot_sum/normal(vectorAfter,0)/normal(vectorBefore,0);//求向量夹角
    if (fabs(angle-1)<alpha)
        rotationAngle = acos(1.0);
    else
        rotationAngle = acos(angle);

    q[0] = cos(rotationAngle/2);
    q[1] = rotationAxis[0]*sin(rotationAngle/2);
    q[2] = rotationAxis[1]*sin(rotationAngle/2);
    q[3] = rotationAxis[2]*sin(rotationAngle/2);
}

void RobotInit() {
}
void *robot_control_thread(void *) {
    double pos[6]= {500.42,148.01,400.79,-2,-2,-8};

    struct sockaddr_in s_in;//server address structure(服务器地址结构)
    struct sockaddr_in c_in;//client address structure(客户端地址结构)
    int l_fd,c_fd;
    socklen_t len;
    char buf[40];//content buff area(内容缓冲区)
    memset((void *)&s_in,0,sizeof(s_in));

    s_in.sin_family = AF_INET;//IPV4 communication domain(通信领域)0.78,-0.02,0.62,-0.09
    s_in.sin_addr.s_addr = INADDR_ANY;//accept any address(接受任何地址)
    s_in.sin_port = htons(PORT);//change port to netchar (改变端口 netchar)

    l_fd = socket(AF_INET,SOCK_STREAM,0);//socket(int domain, int type, int protocol)
    bind(l_fd,(struct sockaddr *)&s_in,sizeof(s_in));
    listen(l_fd, 1);
    cout<<"begin"<<endl;

    c_fd = accept(l_fd,(struct sockaddr *)&c_in,&len);
    read(c_fd, buf, 1);//read the message send by client    o

    double pose_post[7] = {30,-300,600,-0,0,1,0};
    int grab_num=0;
    do{
        double pos_out[7];
        double pos_bef[7];
        double nor[3];
        //read(position_read_fd, pos, sizeof(pos));
        std::cout << "new pos" << std::endl;
        for (int i = 0; i < 7; i++) {
                std::cout << pos[i] << " ";
        }
        std::cout <<std::endl;
        if(pos[5] > 0){
            for(int i=0; i<3; i++){
                pos[i+3] = -pos[i+3];
            }
        }

        double vectorAfter[3];
        for(int i=0; i<3; i++){
            pos_out[i] = pos[i];
            nor[i] = pos[i+3];
            vectorAfter[i] = pos[i+3];
            std::cout<<pos_out[i]<<",";
        }
        normal(nor,1);
        for(int i=0; i<3; i++){
            pos_bef[i] = pos[i] - 100*nor[i];
        }

        double vectorBefore[3]={0,0,1};//输入向量
        double q[4]={0,0,0,0};//四元数初始化b
        compute(q,vectorBefore,vectorAfter);//计算四元数

        for(int i=0; i<4; i++){
            pos_out[i+3] = q[i];
            pos_bef[i+3] = q[i];
            std::cout<<pos_out[i+3]<<",";
        }
        std::cout<<std::endl;

        std::string csf_;
        csf_ = transform_2(pos_bef);
        char out_bef[csf_.length()];
        transform_1(csf_, out_bef);
        int str_len_post = sizeof(out_bef)/sizeof(out_bef[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,out_bef,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        std::string csf;
        csf = transform_2(pos_out);
        char out[csf.length()];
        transform_1(csf, out);
        str_len_post = sizeof(out)/sizeof(out[0]);
        cout<<"str_len is : "<<str_len_post<<endl;
        cout<<"out is : "<<out<<endl;
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,out,str_len_post);             //zhua
        std::cout<<"grab the objct "<<std::endl;
        read(c_fd, buf, 1);//read the message send by client    a
        std::cout<<" buf : "<< buf[0] <<std::endl;

        std::string csf1;
        csf1 = transform_2(pos_bef);
        char lo1[csf1.length()];
        transform_1(csf1, lo1);
        str_len_post = sizeof(lo1)/sizeof(lo1[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,lo1,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        double pos_mul1[7] ={374.76,-0.01,668.00,-0,0,1,0};
        std::string csf2;
        csf2 = transform_2(pos_mul1);
        char lo2[csf2.length()];
        transform_1(csf2, lo2);
        str_len_post = sizeof(lo2)/sizeof(lo2[0]);
        read(c_fd, buf, 1);//read the message send by client    i
        write(c_fd,lo2,str_len_post);
        read(c_fd, buf, 1);//read the message send by client    a

        int post=0;
        do{
            if(post > 2){
                break;
            }

            if(0 == post){
                int which_row = grab_num/3;
                std::cout<<"row: "<<which_row<<std::endl;
                int which_col = grab_num%3;
                std::cout<<"col: "<<which_col<<std::endl;
                pose_post[0] += which_col*110;
                pose_post[1] += which_row*50;
            }else if(1 == post){
                pose_post[2] -= 200;
            }else if(2 == post){
                pose_post[2] += 200;
            }

            read(c_fd, buf, 1);//read the message send by client    i
            std::cout<<" buf : "<< buf <<std::endl;
            std::string csf;
            csf = transform_2(pose_post);
            char out_post[csf.length()];
            transform_1(csf, out_post);
            str_len_post = sizeof(out_post)/sizeof(out_post[0]);
            write(c_fd,out_post,str_len_post);

            if(0 == post){
                std::cout<<"放--"<<std::endl;
            }

            read(c_fd, buf, 1);         //a
            std::cout<<" buf : "<< buf <<std::endl;

            for(int i=0; i<7; i++){
                std::cout<<pose_post[i]<<",";
            }
            std::cout<<std::endl;
            std::cout<<"out is : "<<out_post<<std::endl;
            post++;
        }while('a' == buf[0]);
        std::cout<<" buf : "<< buf <<std::endl;

        read(c_fd, buf, 1);//read the message send by client    i
        std::string csf3 = transform_2(pos_mul1);
        char out_post[csf3.length()];
        transform_1(csf3, out_post);
        str_len_post = sizeof(out_post)/sizeof(out_post[0]);
        write(c_fd,out_post,str_len_post);
        read(c_fd, buf, 1); //                                  a;
        std::cout<<" buf : "<< buf[0] <<std::endl;

        grab_num++;
    }while('a' == buf[0]);

//    struct sockaddr_in s_in;//server address structure(服务器地址结构)
//    struct sockaddr_in c_in;//client address structure(客户端地址结构)
//    int l_fd,c_fd;
//    socklen_t len;
//    char buf[40];//content buff area(内容缓冲区)
//    memset((void *)&s_in,0,sizeof(s_in));

//    s_in.sin_family = AF_INET;//IPV4 communication domain(通信领域)0.78,-0.02,0.62,-0.09
//    s_in.sin_addr.s_addr = INADDR_ANY;//accept any address(接受任何地址)
//    s_in.sin_port = htons(PORT);//change port to netchar (改变端口 netchar)

//    l_fd = socket(AF_INET,SOCK_STREAM,0);//socket(int domain, int type, int protocol)
//    bind(l_fd,(struct sockaddr *)&s_in,sizeof(s_in));
//    listen(l_fd, 1);
//    cout<<"begin"<<endl;

//    c_fd = accept(l_fd,(struct sockaddr *)&c_in,&len);
//    read(c_fd, buf, 1);//read the message send by client    o

//    int grab_num=0;
//    do{
//        double pos_out[7];
//        double pos_bef[7];
//        double nor[3];

//        read(position_read_fd, pos, sizeof(pos));
//        std::cout << "new pos" << std::endl;
//        for (int i = 0; i < 7; i++) {
//                std::cout << pos[i] << " ";
//        }
//        std::cout <<std::endl;
//        if(pos[5] > 0){
//            for(int i=0; i<3; i++){
//                pos[i+3] = -pos[i+3];
//            }
//        }

//        if(0 == grab_num){
//            int n = read(c_fd, buf, 1);//read the message send by client    i
//            cout<<"accept : "<<n<<" buf : "<< buf <<endl;
//            if(!strcmp(buf, "q\n") || !strcmp(buf, "Q\n")){
//                cout << "q pressed\n";
//                close(c_fd);
//                break;
//            }
//        }
//        double vectorAfter[3];
//        for(int i=0; i<3; i++){
//            pos_out[i] = pos[i];
//            nor[i] = pos[i+3];
//            vectorAfter[i] = pos[i+3];
//            std::cout<<pos_out[i]<<",";
//        }
//        normal(nor,1);
//        for(int i=0; i<3; i++){
//            pos_bef[i] = pos[i] +10*nor[i];
//        }

//        double vectorBefore[3]={0,0,1};//输入向量
//        double q[4]={0,0,0,0};//四元数初始化b
//        compute(q,vectorBefore,vectorAfter);//计算四元数

//        for(int i=0; i<4; i++){
//            pos_out[i+3] = q[i];
//            //pos_bef[i+3] = q[i];
//            std::cout<<pos_out[i+3]<<",";
//        }
//        std::cout<<std::endl;

//        std::string csf;
//        csf = transform_2(pos_out);
//        //csf = tranform_2(a);
//        char out[csf.length()];
//        //char out[]={};
//        transform_1(csf, out);
//        int str_len = sizeof(out)/sizeof(out[0]);
//        cout<<"str_len is : "<<str_len<<endl;
//        cout<<"out is : "<<out<<endl;

//        write(c_fd,out,str_len);             //zhua
//        std::cout<<"grab the objct "<<std::endl;
//        read(c_fd, buf, 1);//read the message send by client    a
//        std::cout<<" buf : "<< buf <<std::endl;



////        switch(grab_num){
////            int rows=num%4;
////            case 0:
////                pose_post[0] = 50*rows + pose_post[0];
////            case 1:
////                pose_post[0] = 50*(rows-1) + pose_post[0];
////                pose_post[1] = 50*rows + pose_post[1];
////            case 2:
////                pose_post[0] = 50*(rows-2) + pose_post[0];
////                pose_post[1] = 50*rows + pose_post[1];

////        }

//        //std::cout<<" now begin the procedure of put object"<<std::endl;

//        double pos_mul1[7] ={374.76,-0.01,668.00,-0,0,1,0};
//        double pos_mul2[7] ={230,-500,600,-0,0,1,0};

//        int pos_num = 0;
//        do{
//            if(pos_num > 2){
//                break;
//            }

//            read(c_fd, buf, 1);//read the message send by client    i
//            std::cout<<" buf : "<< buf <<std::endl;
//            int sdg = pos_num%3;
//            std::string csf;
//            if(0 == sdg){
//                csf = transform_2(pos_bef);
//            }else if(1 == sdg){
//                csf = transform_2(pos_mul1);
//            }else if(2 == sdg){
//                csf = transform_2(pos_mul2);
//            }
//            char out_post[csf.length()];
//            transform_1(csf, out_post);
//            int str_len_post = sizeof(out_post)/sizeof(out_post[0]);
//            write(c_fd,out_post,str_len_post);

//            std::cout<<"路径"<< pos_num <<std::endl;

//            read(c_fd, buf, 1);         //a
//            std::cout<<" buf : "<< buf <<std::endl;

//            for(int i=0; i<7; i++){
//                std::cout<<pose_post[i]<<",";
//            }
//            std::cout<<std::endl;
//            std::cout<<"out is : "<<out_post<<std::endl;
//            pos_num++;
//        }while('a' == buf[0]);


//        double pose_post[7] = {230,-347.47,600.00,0,0,1,0};
//        int post=0;
//        do{
//            if(post > 3){
//                break;
//            }

//            read(c_fd, buf, 1);//read the message send by client    i
//            std::cout<<" buf : "<< buf <<std::endl;
//            pose_post[0] += 50;
//            std::string csf;
//            csf = transform_2(pose_post);
//            char out_post[csf.length()];
//            transform_1(csf, out_post);
//            int str_len_post = sizeof(out_post)/sizeof(out_post[0]);
//            write(c_fd,out_post,str_len_post);

//            if(0 == post){
//                std::cout<<"放--"<<std::endl;
//            }

//            read(c_fd, buf, 1);         //a
//            std::cout<<" buf : "<< buf <<std::endl;

//            for(int i=0; i<7; i++){
//                std::cout<<pose_post[i]<<",";
//            }
//            std::cout<<std::endl;
//            std::cout<<"out is : "<<out_post<<std::endl;
//            post++;
//        }while('a' == buf[0]);
//        std::cout<<" buf : "<< buf <<std::endl;
//        //sdojgas++;
//    }while('a' == buf[0]);
}

//visual model and scene
void visualize_pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_src,
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_tgt,
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_final)
{
    //int vp_1, vp_2;
    // Create a PCLVisualizer object
    pcl::visualization::PCLVisualizer viewer("registration Viewer");
    //viewer.createViewPort (0.0, 0, 0.5, 1.0, vp_1);
    // viewer.createViewPort (0.5, 0, 1.0, 1.0, vp_2);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h (pcd_src, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h (pcd_tgt, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h (pcd_final, 0, 0, 255);
    viewer.addPointCloud (pcd_src, src_h, "source cloud");
    viewer.addPointCloud (pcd_tgt, tgt_h, "tgt cloud");
    viewer.addPointCloud (pcd_final, final_h, "final cloud");
    //viewer.addCoordinateSystem(1.0);
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}


// Load and process the cloud in the given PCD file
PointCloud::Ptr loadInputCloud (const std::string &pcd_file)
{
    PointCloud::Ptr cloud = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *cloud);
    return cloud;
}
void removeNANPoints (PointCloud::Ptr cloud)
{
    // Remove NAN points
    std::vector<int> indices; //Reserve removed NAN points indexes
    int before_removeNAN_size = cloud->size();
    pcl::removeNaNFromPointCloud(*cloud,*cloud,indices);
    int after_removeNAN_size = cloud->size();
    std::cout<<"Remove NAN points from "<<before_removeNAN_size<<" to "<<after_removeNAN_size<<endl;
}

void downSampleFilter (PointCloud::Ptr cloud)
{
    // Down sample filter
    const float voxel_grid_size = 0.01f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    int before_downSample_size = cloud->size();
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter (*cloud);
    int after_downSample_size = cloud->size();
    std::cout<<"Down sample filter from "<<before_downSample_size<<" to "<<after_downSample_size<<endl;
}
void downSampleFilter_model (PointCloud::Ptr cloud)
{
    // Down sample filter
    const float voxel_grid_size = 0.0075f;
    pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
    int before_downSample_size = cloud->size();
    vox_grid.setInputCloud (cloud);
    vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vox_grid.filter (*cloud);
    int after_downSample_size = cloud->size();
    std::cout<<"Down sample filter from "<<before_downSample_size<<" to "<<after_downSample_size<<endl;
}

void removeOutlierPoints (PointCloud::Ptr cloud)
{
    // StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    int before_removeOutlier_size = cloud->size();
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud);
    int after_removeOutlier_size = cloud->size();
    std::cout<<"StatisticalOutlierRemoval filter from "<<before_removeOutlier_size<<" to "<<after_removeOutlier_size<<endl;
}

// Compute the surface normals
SurfaceNormals::Ptr computeSurfaceNormals (PointCloud::Ptr cloud)
{
    float normal_radius_ = 0.02f;
    SurfaceNormals::Ptr normals_ = SurfaceNormals::Ptr (new SurfaceNormals);
    SearchMethod::Ptr search_method_xyz_;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud (cloud);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);
    return normals_;
}

// Compute the local feature descriptors
LocalFeatures::Ptr computeLocalFeatures (PointCloud::Ptr cloud,SurfaceNormals::Ptr normals_)
{
    LocalFeatures::Ptr features_ = LocalFeatures::Ptr (new LocalFeatures);
    float feature_radius_ = 0.02f;
    SearchMethod::Ptr search_method_xyz_;
    pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setInputCloud (cloud);
    fpfh_est.setInputNormals (normals_);
    fpfh_est.setSearchMethod (search_method_xyz_);
    fpfh_est.setRadiusSearch (feature_radius_);
    fpfh_est.compute (*features_);
    return features_;
}

int main (int argc, char** argv) {

        std::cout << "Start main" << std::endl;
    int fds[2];
    if (pipe(fds) < 0) {
        std::cout << "pipe failed" << std::endl;
        return 1;
    }
    position_read_fd = fds[0];
    position_write_fd = fds[1];

    if (pipe(fds) < 0) {
        std::cout << "pipe failed" << std::endl;
    }
    ack_read_fd = fds[0];
    ack_write_fd = fds[1];

    RobotInit();

    pthread_t tid;
    pthread_create(&tid, NULL, robot_control_thread, (void *)NULL);

    const float fitness_threshold = 0.0002;
    const int iteration = 300;
    // Load the model clouds specified in the object_templates.txt file
    double start=omp_get_wtime();
    std::string PCDPathPrefix = "./modelPCDFiles/";
        std::cout << "load file ...." << std::endl;
    //读取pcd模型抓取点坐标与法线
    char filename_grab_point[15];
    char filename_grab_normal[15];
    sprintf(filename_grab_point,"grab_points.pcd");
    sprintf(filename_grab_normal,"grab_normals.pcd");
    std::string grab_point_path = PCDPathPrefix + filename_grab_point;
    std::string grab_normal_path = PCDPathPrefix + filename_grab_normal;
    PointCloud::Ptr _grab_points(new PointCloud());
    PointCloud::Ptr _grab_normals(new PointCloud());
    pcl::io::loadPCDFile( grab_point_path , *_grab_points);
    pcl::io::loadPCDFile( grab_normal_path , *_grab_normals);
    std::cout<<"load grab points success"<<std::endl;


#pragma omp parallel for
    for(int i = 0; i < PCD_MODEL_NUM; ++i)
    {
        PointCloud::Ptr model(new PointCloud());
        SurfaceNormals::Ptr model_normals(new SurfaceNormals());
        //LocalFeatures::Ptr model_descriptors;

        //load model pcd file
        char filename[15];
        sprintf(filename, "view%d.pcd", i);
        std::string PCDPath = PCDPathPrefix + filename;
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(PCDPath, *model)==-1)
            continue;
        removeNANPoints(model);
        downSampleFilter_model(model);
        removeOutlierPoints(model);
        model_array[i]=model;
        model_normals = computeSurfaceNormals(model);
        model_normals_array[i] = model_normals;
        // model_normals.push_back(model_normal);
        model_descriptors_array[i] = computeLocalFeatures(model,model_normals);
    }
    //模型分类
    std::vector<int> label={0,10,32,50,53,0,1,2,3,4,5,6,7,24,25,26,27,28,29,30,31,48,52,56,60,64,68,72,76,10,13,18,21,33,34,37,38,41,42,45,46,61,65,77,54,58,70,74,32,35,36,39,40,43,44,47,50,51,62,63,66,67,78,79,53,55,57,59,69,71,73,75};
    cout<<"load model size : "<< label.size() <<endl;

    double load_model_time=omp_get_wtime();
    cout<<"load_model_time: "<<load_model_time-start<<" s"<<endl;

    //get camera cloud
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();
    // bool is_continue;
    int num=0;
    char ret;
    do {
        // Application still alive?
        //bool is_continue=false;
        //每次获取一个点云然后处理
        cout<<"start get point cloud from camera"<<std::endl;

        double load_scene_time=omp_get_wtime();

        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        cout<< "Points Sizes: " <<points.size()<<endl;
        auto vertices = points.get_vertices();
        auto width = depth.get_width();
        auto height = depth.get_height();
        cout<< "width: " <<width<<endl;
        cout<< "height: " <<height<<endl;

        PointCloud::Ptr cloud(new PointCloud());

        // Fill in the cloud data
        cloud->width    = width;
        cloud->height   = height;
        cloud->is_dense = false;
        cloud->points.resize (cloud->width * cloud->height);

        size_t size = cloud->points.size();
#pragma omp parallel for
        for (size_t i = 0; i < size; i++) {
            cloud->points[i].x = vertices[i].x;
            cloud->points[i].y = vertices[i].y;
            cloud->points[i].z = vertices[i].z;
        }
        cout<<"successfully generate cloud"<<std::endl;
        double load_scene=omp_get_wtime();
        cout<<load_scene-load_model_time<<endl;
        pcl::io::savePCDFile("scene.pcd", *cloud);
        PointCloud::Ptr scene(cloud);
        removeNANPoints(scene);
        downSampleFilter(scene);
        removeOutlierPoints(scene);

        double load_normal=omp_get_wtime();
        SurfaceNormals::Ptr scene_normals (new SurfaceNormals());
        scene_normals = computeSurfaceNormals(scene);
        double load_normal_end=omp_get_wtime();
        LocalFeatures::Ptr scene_descriptors(new LocalFeatures()) ;
        scene_descriptors= computeLocalFeatures(scene,scene_normals);
        double load_fpfh=omp_get_wtime();
        cout<<"load_normal"<<load_normal_end-load_normal<<endl;
        cout<<"load fpfh"<<load_fpfh-load_normal_end<<endl;
        /*
           std::vector<float> align_score;
           std::vector<int> label1;
        //std::vector<PointCloud::Ptr> sac_ia_align_points;
        bool  judge = false ;

        // #pragma omp parallel for
#pragma omp parallel for num_threads(4)
for (size_t i = 0; i < 5; ++i)
{
if( judge == true)
continue ;
if( judge == false)
{
pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
scia.setMaximumIterations (iteration);   //设置粗匹配的 最大迭代次数
scia.setMaxCorrespondenceDistance(50); // 设置最大的对应距离
scia.setInputCloud (model_array[label[i]]);
scia.setSourceFeatures (model_descriptors_array[label[i]]);

scia.setInputTarget (scene);
scia.setTargetFeatures (scene_descriptors);

PointCloud::Ptr sac_ia_align_cloud (new PointCloud);
scia.align (*sac_ia_align_cloud);
        //sac_ia_align_points.push_back(sac_ia_align_cloud);
        align_score.push_back((float) scia.getFitnessScore ());
        label1.push_back(label[i]);

        std::cout  <<"SAC-IA model "<< label[i] <<" has converged:"<<scia.hasConverged()<<" with score: "<<scia.getFitnessScore ()<<endl;

        if((float) scia.getFitnessScore ()<  thread )
        judge = true;
        }
        }
        // Find the best template alignment
        // Find the template with the best (lowest) fitness score
        float lowest_score = std::numeric_limits<float>::infinity ();
        int best_model_index = -1;
        int label2=-1;
        for (size_t i = 0; i < align_score.size (); ++i)
        {
        if (align_score[i] < lowest_score)
        {
        lowest_score = align_score[i];
        best_model_index = (int) label[i];
        label2=i;
        }
        }
        printf ("Best fitness is model view%d with score: %f\n", best_model_index,lowest_score);

        int initinal_number;
        int end_number;
        start_to_end(label2,initinal_number,end_number);
        */
        //std::vector<PointCloud::Ptr> sac_ia_align_points_linin;
        //            std::vector<Eigen::Matrix4f>  sac_trans;
        //            Eigen::Matrix4f model_sac_trans;
        //            std::vector<PointCloud::Ptr> sac_ia_align_points;
        bool  casea = false ;
        //            std::vector<int> label_index;
        //            std::vector<float> align_score_class;
        float lowest_score = std::numeric_limits<float>::infinity ();
        int best_model_index = 0;
        Eigen::Matrix4f best_sac_trans;
        PointCloud::Ptr best_sacia_align_cloud (new PointCloud);



#pragma omp parallel for num_threads(4)
        for (size_t i = 0; i <PCD_MODEL_NUM ; ++i)
        {
            if (casea == true) {
                continue;
            }
            pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
            scia.setMaximumIterations (iteration);   //设置粗匹配的 最大迭代次数
            scia.setMaxCorrespondenceDistance(50);   // 设置最大的对应距离

            scia.setInputCloud (model_array[i]);
            scia.setSourceFeatures (model_descriptors_array[i]);

            scia.setInputTarget (scene);
            scia.setTargetFeatures (scene_descriptors);

            PointCloud::Ptr sac_ia_align_cloud (new PointCloud);
            scia.align (*sac_ia_align_cloud);
            //sac_ia_align_points_linin.push_back(sac_ia_align_cloud);
            //下边的pushback需要加锁处理
            //#pragma omp critical
            //                {
            //                    align_score_class.push_back((float)scia.getFitnessScore ());
            //                    sac_ia_align_points.push_back(sac_ia_align_cloud);
            //                    model_sac_trans=scia.getFinalTransformation();
            //                    label_index.push_back(i);
            //                }
            auto score = scia.getFitnessScore();
        #pragma omp critical
            {

                if (scia.getFitnessScore () < lowest_score)
                {
                    lowest_score = score;
                    best_model_index = (int) i;
                    best_sac_trans = scia.getFinalTransformation();
                    best_sacia_align_cloud = sac_ia_align_cloud;
                }
            }


            std::cout  <<"SAC-IA model "<< i <<" has converged:"<<scia.hasConverged()<<" with score: "<<scia.getFitnessScore ()<<endl;

            if(score < fitness_threshold) {
                casea = true;
            }
        }

        //      // Find the best template alignment
        //      // Find the template with the best (lowest) fitness score
        //      float lowest_score_new = align_score_class[0];
        //      int best_model_index_new = 0;
        //      Eigen::Matrix4f model_sac_trans;
        //      std::cout<<align_score_class.size ()<<std::endl;
        //      std::cout<<sac_trans.size ()<<std::endl;
        //      for (size_t i = 0; i < align_score_class.size (); ++i)
        //      {
        //         if (align_score_class[i] < lowest_score_new)
        //         {
        //             std::cout<<sac_trans[i]<<std::endl;
        // lowest_score_new = align_score_class[i];
        // best_model_index_new = (int) label_index[i];
        //             model_sac_trans=sac_trans[i];
        //             std::cout<<1111<<endl;
        //         }500.42,148.01,400.79,0.169102,0.696923,-0.696923,0
        //      }
        //      std::cout<<sac_trans.size ()<<std::endl;
        //      printf ("Best fitness is model view%d with score: %f\n", best_model_index_new,lowest_score_new);

        // Find the best template alignmentmovement(pos,vv=30);
        // Find the template with the best (lowest) fitness score
        //            float lowest_score = std::numeric_limits<float>::infinity ();
        //            int best_model_index = 0;
        //            for (size_t i = 0; i < align_score_class.size (); ++i)
        //            {
        //                if (align_score_class[i] < lowest_score)
        //                {
        //                  lowest_score = align_score_class[i];
        //                  best_model_index = (int) i;
        //                }
        //            }
        printf ("Best fitness is model %d with score: %f\n", best_model_index,lowest_score);


        double model_align_time=omp_get_wtime();
        cout<<"model_align_time: "<<model_align_time-load_scene_time<<" s"<<endl;

        // PointCloud::Ptr model_sac (new PointCloud);
        PointCloud::Ptr icp_result (new PointCloud);

        // pcl::transformPointCloud (*model_array[best_model_index_new], *model_sac, model_sac_trans);
        Eigen::Matrix4f icp_best_trans;
        float icp_best_score=1;
        float eps=1e-10;
        double maxitera=50;
        double fiteps=0.2;
        do{
            PointCloud::Ptr icp_pc (new PointCloud);
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(best_sacia_align_cloud);
            icp.setInputTarget(scene);
            //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance (0.04);
            icp.setMaximumIterations (300);                          //50
            icp.setTransformationEpsilon (1e-15);                   //1e-10
            icp.setEuclideanFitnessEpsilon (0.001);                   //0.2
            icp_best_trans=icp.getFinalTransformation();
            icp.align(*icp_pc);

            float icp_score = icp.getFitnessScore();
            if (abs(icp_score - icp_best_score) < 0.00000001) break;

            if(icp_best_score > icp.getFitnessScore()){
                icp_best_score=icp.getFitnessScore();
                icp_best_trans=icp.getFinalTransformation();
                icp_result = icp_pc;
            }

            eps /= 10;
            maxitera *= 2;
            fiteps /=10;
            std::cout << "ICP get score: " << icp_best_score << std::endl;
        }while(icp_best_score>0.0003);
        std::cout << "ICP best score: " << icp_best_score << std::endl;

        //坐标转换：model->scene->world
        Eigen::Matrix4f transformation = icp_best_trans;
        double icp_align_time=omp_get_wtime();
        cout<<"icp_align_time: "<<icp_align_time-model_align_time<<" s"<<endl;
        cout<<"total_time: "<<icp_align_time-load_scene_time<<" s"<<endl;

        //Eigen::Matrix4f trans_cameraToWorld(4,4);			//深度相机坐标系转基座坐标系 R、T
        Eigen::Matrix4f trans_cameraToWorld;			//深度相机坐标系转基座坐标系 R、T
        //trans_cameraToWorld << -6.24428055e-03,9.48115918e-01,-2.93495458e-01,7.45805912e+02,1.00920853e+00,3.52099308e-02,4.88279811e-02,2.65093137e+01,-3.73284675e-02,-3.39979155e-01,-9.81241867e-01,9.94411352e+02,0.00000000e+00,-1.08420217e-19,8.67361738e-19,1.00000000e+00;
        //trans_cameraToWorld << -9.49916884e-03,1.03921097e+00,9.35016081e-02,7.35659509e+02,1.06147020e+00,-6.80344185e-01,-8.36702492e-01,2.32063988e+02,-7.63207147e-02,1.61278728e+00,1.00659932e+00,2.57715554e+02,3.46944695e-18,-5.55111512e-17,-2.77555756e-17,1.00000000e+00;
        trans_cameraToWorld << 4.36143742e-02 ,  1.03571471e+00  ,-4.23509811e-03 ,  7.46869934e+02,9.79732208e-01 , -5.42240272e-02 , -9.80312327e-03 , -6.75789098e+01,1.76851377e-02 , -8.41952885e-02 , -1.03952415e+00  , 9.95094578e+02,2.16840434e-19  ,-8.67361738e-19 ,  0.00000000e+00  , 1.00000000e+00;
        PointCloud::Ptr grab = _grab_points;
        PointCloud::Ptr grab_translated (new PointCloud());
        PointCloud::Ptr grab_translated_icp (new PointCloud());
        PointCloud::Ptr grab_translated_world (new PointCloud());	//基座坐标 point
        std::cout<<*grab<<std::endl;
        std::cout<<*grab_translated<<std::endl;
        pcl::transformPointCloud (*grab, *grab_translated,best_sac_trans );
        pcl::transformPointCloud (*grab_translated, *grab_translated_icp, transformation);

        PointCloud::Ptr icp_m2mm (new PointCloud());
        PointCloud::Ptr icp_world (new PointCloud());
        PointCloud::Ptr grab_translated_icp_m2mm (new PointCloud());
        for (size_t i=0;i<grab_translated_icp->points.size();++i)
        {
            pcl::PointXYZ p;
            p.x = 1000*grab_translated_icp->points[i].x;
            p.y = 1000*grab_translated_icp->points[i].y;
            p.z = 1000*grab_translated_icp->points[i].z;
            grab_translated_icp_m2mm->points.push_back(p);
        }
        for (size_t i=0;i<icp_result->points.size();++i)
        {
                pcl::PointXYZ _p;
                _p.x = 1000*icp_result->points[i].x;
                _p.y = 1000*icp_result->points[i].y;
                _p.z = 1000*icp_result->points[i].z;
                icp_m2mm->points.push_back(_p);
        }
        pcl::transformPointCloud (*grab_translated_icp_m2mm, *grab_translated_world, trans_cameraToWorld);
        /*/pcl::transformPointCloud (*icp_m2mm, *icp_world, trans_cameraToWorld);

        pcl::PointXYZ grab_p;
        grab_p=icp_world->points[0];
        for(size_t i=1;i < icp_world->points.size();++i)
        {
                if(grab_p.z < icp_world->points[i].z)
                {
                        grab_p=icp_world->points[i];
                }
        }
        */

        PointCloud::Ptr grab_normals=_grab_normals;
        PointCloud::Ptr grab_normals_translated (new PointCloud ());
        PointCloud::Ptr grab_normals_translated_icp (new PointCloud ());
        PointCloud::Ptr grab_normals_translated_world (new PointCloud ());	//基座坐标 normal

        Eigen::Matrix4f using_translationInvTran = (best_sac_trans.inverse()).transpose();
        Eigen::Matrix4f transformationInvTran = (transformation.inverse()).transpose();
        Eigen::Matrix4f trans_cameraToWorldInvTran = (trans_cameraToWorld.inverse()).transpose();

        pcl::transformPointCloud (*grab_normals, *grab_normals_translated, using_translationInvTran);
        pcl::transformPointCloud (*grab_normals_translated, *grab_normals_translated_icp, transformationInvTran);
        pcl::transformPointCloud (*grab_normals_translated_icp, *grab_normals_translated_world, trans_cameraToWorldInvTran);


        std::string TestPathPrefix = "./modelPCDFiles/";
        char test_grab_point[15];
        char test_grab_normal[15];
        sprintf(test_grab_point,"grab_points_test.pcd");
        sprintf(test_grab_normal,"grab_normals_test.pcd");
        std::string test_grab_point_path=TestPathPrefix+test_grab_point;
        std::string test_grab_normal_path=TestPathPrefix+test_grab_normal;
        pcl::io::savePCDFile(test_grab_point_path,*grab_translated_icp);
        pcl::io::savePCDFile(test_grab_normal_path,*grab_normals_translated_icp);

        std::ofstream outfile("grabPoint_world.txt");
        outfile << grab_translated_world->points[0].x << " ";
        outfile << grab_translated_world->points[0].y << " ";
        outfile << grab_translated_world->points[0].z << " ";
        outfile << grab_normals_translated_world->points[0].x <<" ";
        outfile << grab_normals_translated_world->points[0].y<<" ";
        outfile << grab_normals_translated_world->points[0].z<<"\n";
        outfile.close();
/*
        double pos[7];
        pos[0] = grab_translated_world->points[0].x;
        pos[1] = grab_translated_world->points[0].y;
        pos[2] = grab_translated_world->points[0].z;
        pos[3] = grab_normals_translated_world->points[0].x;
        pos[4] = grab_normals_translated_world->points[0].y;
        pos[5] = grab_normals_translated_world->points[0].z;
        pos[6] = num;
        num++;
        cout<<"fr:"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<" fg:"<<pos[3]<<" "<<pos[4]<<"  "<<pos[5]<<endl;
        if(pos[5]<0)
        {
            pos[3] = -pos[3];
            pos[4] = -pos[4];
            pos[5] = -pos[5];
        }
*/

        cout<<"total_time: "<<icp_align_time-load_scene_time<<" s"<<endl;

//        //可视化化
//        pcl::PointXYZ grab_point_;
//        grab_point_.x=grab_translated_icp->points[0].x;
//        grab_point_.y=grab_translated_icp->points[0].y;
//        grab_point_.z=grab_translated_icp->points[0].z;
//        std::cout<<grab_translated_icp_m2mm->points[0].x<<","<<grab_translated_icp_m2mm->points[0].y<<","<<grab_translated_icp_m2mm->points[0].z<<std::endl;
//        pcl::Normal _normal;
//        _normal.normal_x=grab_normals_translated_icp->points[0].x;
//        _normal.normal_y=grab_normals_translated_icp->points[0].y;
//        _normal.normal_z=grab_normals_translated_icp->points[0].z;
//        pcl::PointCloud<pcl::Normal>::Ptr grab_nl_ (new pcl::PointCloud<pcl::Normal>());
//        pcl::PointCloud<pcl::PointXYZ>::Ptr grab_pt_ (new pcl::PointCloud<pcl::PointXYZ>());
//        grab_pt_->points.push_back(grab_point_);
//        grab_nl_->points.push_back(_normal);

//        pcl::visualization::PCLVisualizer viewer("PCL Viewer");

//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_array_color_handler (model_array[0], 0, 0, 255);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_handler (scene, 255, 0, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_result_color_handler (icp_result, 0, 255, 0);

//        viewer.addPointCloud(model_array[0],model_array_color_handler, "model_cloud");
//        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_cloud");

//        viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");
//        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud");

//        viewer.addPointCloud(icp_result,icp_result_color_handler, "icp_result");
//        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_result");

//        viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (grab_pt_, grab_nl_, 1, 0.2, "normals");

//        while(!viewer.wasStopped ())
//        {
//           viewer.spinOnce (1);
//        }

        //double pos[7] = {644.47,-52.22,737.24,0.78,-0.02,0.62,-0.09};
        double pos[6] = {500.42,148.01,400.79,-2,-2,-8};
        write(position_write_fd, pos, sizeof(pos));
    } while (read(ack_read_fd, &ret, 1));
    pthread_join(tid, NULL);
    return 0;
}

