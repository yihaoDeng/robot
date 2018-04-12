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
#include "libClient.h"
#include <unistd.h>
#include<cstdlib>
#include<ctime>
#include <sstream>
#include<cmath>
#define random(a,b) (rand()%(b-a+1)+a)
//////////////////////////////robot

// A bit of shorthand
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

#define PCD_MODEL_NUM 80

PointCloud::Ptr model_array[PCD_MODEL_NUM];
SurfaceNormals::Ptr model_normals_array[PCD_MODEL_NUM];
LocalFeatures::Ptr model_descriptors_array[PCD_MODEL_NUM];
PointCloud::Ptr grab_point_array[PCD_MODEL_NUM];
PointCloud::Ptr grab_point_normal_array[PCD_MODEL_NUM];

std::mutex mutex;

//initial for robot
void init()
{
    systemInit();
    cout << "press any key to continue!" << endl;
    getchar();
    terminalStruct toolFrame({45,0,45,180,0,180});
    dex.modifyToolFrame(toolFrame);
    dex.setServoState(SWITCHON);
}
void movement(double pos[], double v = 60)
{
    //法向量归一化
    double denominator = sqrt(pow(pos[3],2) + pow(pos[4],2) + pow(pos[5],2));
    pos[3] /= denominator;
    pos[4] /= denominator;
    pos[5] /= denominator;
    terminalStruct t,t_temp;
    normStruct n;
    thetaphiStruct tps;
    for(int i=0; i<3; i++)
    {
        t.terminalPos[i] = pos[i];
        t_temp.terminalPos[i] = pos[i]+80*pos[i+3];
    }
    for(int i=0; i<3; i++)
    {
        n.normVec[i] = -pos[i+3];
    }
    tps = getThetaPhi(n);
    t.terminalPos[3] = tps.angle[0];
    t.terminalPos[4] = tps.angle[1];
    t.terminalPos[5] = 0;

    t_temp.terminalPos[3] = tps.angle[0];
    t_temp.terminalPos[4] = tps.angle[1];
    t_temp.terminalPos[5] = 0;

    for (int i=0;i<3;i++)
        cout<<t.terminalPos[i]<<"   ";
    for (int i=0;i<3;i++)
    {
        cout<<n.normVec[i]<<"  ";
    }

    dex.moveLine(t_temp, 500);
    dex.waitCommandEnd();

    dex.moveLine(t, 300);
    dex.waitCommandEnd();
    // open mouth
    SWITCHSTATE io[16];
    dex.getInputState(io);
    for(int i = 0; i<16; i++)
        io[i] = SWITCHON;
    dex.setOutputState(io);
    // go up
    terminalStruct tt;
    dex.getCurrentTerminal(tt);
    tt.terminalPos[2] = 700;
    tt.terminalPos[3] = 0;
    tt.terminalPos[4] = 0;
    tt.terminalPos[5] = 0;
    dex.moveLine(tt, 500);
    dex.waitCommandEnd();


    // move joint
    /*jointStruct j;
    dex.getCurrentJoint(j);
    j.jointAngle[0]=60;
    dex.moveJoint(j,0.1);
    dex.waitCommandEnd();*/
    // setting target
    terminalStruct target;
    dex.getCurrentTerminal(target);
    target.terminalPos[0] = 150;
    target.terminalPos[1] = 600;
    target.terminalPos[2] = 500;
    dex.moveLine(target,500);
    dex.waitCommandEnd();
    // close mouth
    for(int i = 0; i<16; i++)
        io[i] = SWITCHOFF;
    dex.setOutputState(io);
    sleep(1);

    dex.returnZero(0.2);
    dex.waitCommandEnd();
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

void start_to_end(int &lab,int &st,int &ends){

    if (lab == 0)
    {
        st = 5;
        ends = 28;
    }
    else if(lab ==1)
    {
        st = 29;
        ends = 47;
    }
    else if(lab ==2)
    {
        st = 48;
         ends = 55;
    }
    else if(lab ==3)
    {
        st = 56;
         ends = 63;
    }
    else
    {
        st = 64;
         ends = 71;
    }


}

void read_grab_points(pcl::PointCloud<pcl::PointXYZ>::Ptr grab_point_array[PCD_MODEL_NUM],pcl::PointCloud<pcl::PointXYZ>::Ptr grab_point_normal_array[PCD_MODEL_NUM])
{
        std::vector<std::string> vec;
        std::ifstream myfile("grab_points.txt");
        std::string temp;
    if (!myfile.is_open())
    {
        std::cout << "未成功打开文件" << std::endl;
		exit(-1);
    }
    while(getline(myfile,temp))
    {
        vec.push_back(temp);
    }
    for (std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); it++)
    {
        //std::cout << *it << std::endl;
    }
        int n=0;
    for (std::vector<std::string>::iterator it = vec.begin(); it != vec.end(); it++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr normals_(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ p;
        pcl::PointXYZ nl;

        std::istringstream record(*it);
        float s;
        int m=0;
        while (record >> s)
{
        //std::cout<<s<<std::endl;

        switch (m)
        {
                case 0:
                        p.x=s;
                        break;
                case 1:
                        p.y=s;
                        break;
                case 2:
                        p.z=s;
                        //std::cout<<"point:"<<p.x <<p.y<<p.z<<std::endl;
                        cloud_->push_back(p);
                        break;
                case 3:
                        nl.x=s;
                        break;
                case 4:
                        nl.y=s;
                        break;
                case 5:
                        nl.z=s;
                        normals_->push_back(nl);
                        break;
                case 6:
                        p.x=s;
                        break;
                case 7:
                        p.y=s;
                        break;
                case 8:
                        p.z=s;
                        cloud_->push_back(p);
                        break;
                case 9:
                        nl.x=s;
                        break;
                case 10:
                        nl.y=s;
                        break;
                case 11:
                        nl.z=s;
                        normals_->push_back(nl);
                        break;
                case 12:
                        p.x=s;
                        break;
                case 13:
                        p.y=s;
                        break;
                case 14:
                        p.z=s;
                        cloud_->push_back(p);
                        break;
                case 15:
                        nl.x=s;
                        break;
                case 16:
                        nl.y=s;
                        break;
                case 17:
                        nl.z=s;
                        normals_->push_back(nl);
                        break;
                default :
                        std::cout<<"excuese me?"<<std::endl;
                        break;

        }//switch end
        //std::cout<<"m :"<<m<<std::endl;
        m++;
}//while end
        int width=1;
        int height=cloud_->points.size();
        cloud_->resize(width*height);

        grab_point_array[n]=cloud_;
        grab_point_normal_array[n]=normals_;
/*
        std::cout<<cloud_->points.size()<<std::endl;
        std::cout<<cloud_->points[0].x<<std::endl;
        std::cout<<normals_->points.size()<<std::endl;
        std::cout<<normals_->points[0].normal_x<<std::endl;

        std::cout<<grab_point_array[n]->points.size()<<std::endl;
        std::cout<<grab_point_array[n]->points[0].x<<std::endl;
        std::cout<<grab_point_normal_array[n]->points.size()<<std::endl;
        std::cout<<grab_point_normal_array[n]->points[0].normal_x<<std::endl;
*/
        n++;
    } //for end

        //std::cout<<"success "<<std::endl;
/*
        for (size_t i=0;i<PCD_MODEL_NUM;++i)
        {
                //std::cout<<i<<"-Number:"<<*grab_point_array[i]<<std::endl;
                for (size_t j=0;j<grab_point_array[i]->points.size();j++)
                {
                        std::cout<<"model"<<i<<": "<<grab_point_array[i]->points[j].x<<","<<grab_point_array[i]->points[j].y<<","<<grab_point_array[i]->points[j].z<<std::endl;
                }
        }
*/

        myfile.close();
}

int
main (int argc, char** argv)
{
        init();
        const float thread = 0.0002;
        const int iteration = 300;
	// Load the model clouds specified in the object_templates.txt file
        double start=omp_get_wtime();
        std::string PCDPathPrefix = "./modelPCDFiles/";
        //读取pcd模型抓取点坐标与法线
        read_grab_points(grab_point_array,grab_point_normal_array);
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
        downSampleFilter(model);
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
     for(int i=0;i<10;i++){
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
            //pcl::io::savePCDFile("scene.pcd",cloud);
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

				if(score < thread) {
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
			//         }
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

			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputSource(best_sacia_align_cloud);
			icp.setInputTarget(scene);
			//Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
			icp.setMaxCorrespondenceDistance (0.04);
                        icp.setMaximumIterations (300);                          //50
                        icp.setTransformationEpsilon (1e-15);                   //1e-10
                        icp.setEuclideanFitnessEpsilon (0.001);                   //0.2
			icp.align(*icp_result);

			std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

			//坐标转换：model->scene->world
			Eigen::Matrix4f transformation = icp.getFinalTransformation();
			double icp_align_time=omp_get_wtime();
			cout<<"icp_align_time: "<<icp_align_time-model_align_time<<" s"<<endl;
			cout<<"total_time: "<<icp_align_time-load_scene_time<<" s"<<endl;

			//Eigen::Matrix4f trans_cameraToWorld(4,4);			//深度相机坐标系转基座坐标系 R、T
			Eigen::Matrix4f trans_cameraToWorld;			//深度相机坐标系转基座坐标系 R、T
                        trans_cameraToWorld << -6.24428055e-03,9.48115918e-01,-2.93495458e-01,7.45805912e+02,1.00920853e+00,3.52099308e-02,4.88279811e-02,2.65093137e+01,-3.73284675e-02,-3.39979155e-01,-9.81241867e-01,9.94411352e+02,0.00000000e+00,-1.08420217e-19,8.67361738e-19,1.00000000e+00;
			PointCloud::Ptr grab = grab_point_array[best_model_index];
			PointCloud::Ptr grab_translated (new PointCloud());
			PointCloud::Ptr grab_translated_icp (new PointCloud());
			PointCloud::Ptr grab_translated_world (new PointCloud());	//基座坐标 point
			std::cout<<*grab<<std::endl;
			std::cout<<*grab_translated<<std::endl;
			pcl::transformPointCloud (*grab, *grab_translated,best_sac_trans );
			pcl::transformPointCloud (*grab_translated, *grab_translated_icp, transformation);

			PointCloud::Ptr grab_translated_icp_m2mm (new PointCloud());
			for (size_t i=0;i<grab_translated_icp->points.size();++i)
			{
				pcl::PointXYZ p;
				p.x = 1000*grab_translated_icp->points[i].x;
				p.y = 1000*grab_translated_icp->points[i].y;
				p.z = 1000*grab_translated_icp->points[i].z;
				grab_translated_icp_m2mm->points.push_back(p);
			}
			pcl::transformPointCloud (*grab_translated_icp_m2mm, *grab_translated_world, trans_cameraToWorld);

			PointCloud::Ptr grab_normals=grab_point_normal_array[best_model_index];
			PointCloud::Ptr grab_normals_translated (new PointCloud ());
			PointCloud::Ptr grab_normals_translated_icp (new PointCloud ());
			PointCloud::Ptr grab_normals_translated_world (new PointCloud ());	//基座坐标 normal

			Eigen::Matrix4f using_translationInvTran = (best_sac_trans.inverse()).transpose();
			Eigen::Matrix4f transformationInvTran = (transformation.inverse()).transpose();
			Eigen::Matrix4f trans_cameraToWorldInvTran = (trans_cameraToWorld.inverse()).transpose();

			pcl::transformPointCloud (*grab_normals, *grab_normals_translated, using_translationInvTran);
			pcl::transformPointCloud (*grab_normals_translated, *grab_normals_translated_icp, transformationInvTran);
			pcl::transformPointCloud (*grab_normals_translated_icp, *grab_normals_translated_world, trans_cameraToWorldInvTran);

			std::ofstream outfile("grabPoint_world.txt");
			outfile<<grab_translated_world->points[0].x<<" ";
			outfile<<grab_translated_world->points[0].y<<" ";
			outfile<<grab_translated_world->points[0].z<<" ";
			outfile<<grab_normals_translated_world->points[0].x<<" ";
			outfile<<grab_normals_translated_world->points[0].y<<" ";
			outfile<<grab_normals_translated_world->points[0].z<<"\n";
			outfile.close();

                        double pos[6],vv;
                        pos[0] = grab_translated_world->points[0].x;
                        pos[1] = grab_translated_world->points[0].y;
                        pos[2] = grab_translated_world->points[0].z+5;
                        pos[3] = grab_normals_translated_world->points[0].x;
                        pos[4] = grab_normals_translated_world->points[0].y;
                        pos[5] = grab_normals_translated_world->points[0].z;
                        cout<<"fr"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<"fg"<<pos[3]<<" "<<pos[4]<<"  "<<pos[5]<<endl;
                        if(pos[5]<0)
                        {
                            pos[3] = -pos[3];
                            pos[4] = -pos[4];
                            pos[5] = -pos[5];
                        }


			cout<<"total_time: "<<icp_align_time-load_scene_time<<" s"<<endl;


			//可视化
                        pcl::PointXYZ grab_point_;
			grab_point_.x=grab_translated_icp->points[0].x;
			grab_point_.y=grab_translated_icp->points[0].y;
			grab_point_.z=grab_translated_icp->points[0].z;
			std::cout<<grab_translated_icp_m2mm->points[0].x<<","<<grab_translated_icp_m2mm->points[0].y<<","<<grab_translated_icp_m2mm->points[0].z<<std::endl;
			pcl::Normal _normal;
			_normal.normal_x=grab_normals_translated_icp->points[0].x;
			_normal.normal_y=grab_normals_translated_icp->points[0].y;
			_normal.normal_z=grab_normals_translated_icp->points[0].z;
			pcl::PointCloud<pcl::Normal>::Ptr grab_nl_ (new pcl::PointCloud<pcl::Normal>());
			pcl::PointCloud<pcl::PointXYZ>::Ptr grab_pt_ (new pcl::PointCloud<pcl::PointXYZ>());
			grab_pt_->points.push_back(grab_point_);
			grab_nl_->points.push_back(_normal);

			pcl::visualization::PCLVisualizer viewer("PCL Viewer");

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_array_color_handler (model_array[best_model_index], 0, 0, 255);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_color_handler (scene, 255, 0, 0);
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> icp_result_color_handler (icp_result, 0, 255, 0);

			viewer.addPointCloud(model_array[best_model_index],model_array_color_handler, "model_cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "model_cloud");

			viewer.addPointCloud(scene, scene_color_handler, "scene_cloud");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scene_cloud");

			viewer.addPointCloud(icp_result,icp_result_color_handler, "icp_result");
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "icp_result");

			viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (grab_pt_, grab_nl_, 1, 0.2, "normals");

			//viewer.addArrow<pcl::PointXYZ, pcl::PointXYZ>(grab_normal_,grab_point_,0,1,0,false,"normal_arrow");

			while(!viewer.wasStopped ())
			{
				viewer.spinOnce ();
                        }


                        movement(pos,vv=30);
			//visualize_pcd(model_array[best_model_index],scene,icp_result);

                        //std::cout<<"是否继续matching，请输入（0/1）："<<std::endl;
                        //int is_continue;
                        //std::cin>>is_continue;
            }
     return 0;
}

