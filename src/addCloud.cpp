#include <iostream>
#include <cmath>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "./../include/addCloud.hpp"

namespace addCloud {

    SumCloud::SumCloud(const std::string &file_path, 
                       const std::string &matrix_path,
                       const std::string &code) : 
        sum_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        total_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        filepath (file_path),
        matrixpath (matrix_path),
        file_code (code),
        R (Eigen::Matrix4f::Identity()),
        count (0)
    {
        rote = new GetRotationVector();
    }

    SumCloud::~SumCloud() {
        delete rote;
    }

    void SumCloud::remove_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        boost::shared_ptr<pcl::SACSegmentation<pcl::PointXYZ>> seg (new pcl::SACSegmentation<pcl::PointXYZ>);
    
        seg->setOptimizeCoefficients(true);
        seg->setModelType(pcl::SACMODEL_PLANE);
        seg->setMethodType(pcl::SAC_RANSAC);
        seg->setDistanceThreshold(0.15);
        seg->setInputCloud(cloud);
        seg->segment(*inliers, *coefficients);

        std::cout << cloud->points.size() << std::endl;

        if(inliers->indices.size() != 0)
        {
            for (size_t i=0; i<inliers->indices.size(); ++i)
            {
                cloud->points[inliers->indices[i]].x = 0.0;
                cloud->points[inliers->indices[i]].y = 0.0;
                cloud->points[inliers->indices[i]].z = 0.0;
            }
        }
    }


    void SumCloud::outlineFilter() {

        std::cout << "outlier filter" << std::endl;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(5);
        sor.setStddevMulThresh(0.7);
        sor.filter(*cloud);
        std::cout << "CLOUD SIZE : " << cloud->points.size() << std::endl;
    }

    void SumCloud::voxelization_filter() {

        float voxel_size = 0.0128;
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(total_cloud);
        sor.setLeafSize(voxel_size, voxel_size, voxel_size);
        sor.filter(*total_cloud);
    }

    int SumCloud::getfileNum()
    {
        namespace fs = boost::filesystem;
        if(!fs::exists(filepath) || !fs::is_directory(filepath))
        {
            std::cout << "file path misstake !!" << std::endl;
            exit(0);
        }

        fs::directory_iterator last;
        for(fs::directory_iterator pos(filepath); pos!=last; ++pos)
        {
            ++count;
        }

        std::cout << "files num : " << std::to_string(count) << std::endl;

        return count;
    }

    

    void SumCloud::make_total_cloud()
    {
        int num = getfileNum();

        for(int i=0; i<num; i++)
        {
            std::string cloud_name = filepath + file_code + std::to_string(i) + ".ply"; 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile(cloud_name, *cloud);
            remove_plane(cloud);

            for(int j=i; j>0; j--)
            {
	            std::string matrix_name = matrixpath 
                                        + "matrix" 
                                        + std::to_string(j-1) 
                                        + "_" 
                                        + std::to_string(j) 
                                        + ".txt";

                rote->get_matrix4x4(matrix_name);
                R = rote->R;
                pcl::transformPointCloud(*cloud, *cloud, R);
            }
            //outlineFilter();
            *total_cloud += *cloud; 
            //voxelization_filter();
        }
         
        std::string save_name = "/mnt/container-data/katori_0906/ply_total/after1-2.ply";
        pcl::io::savePLYFileASCII(save_name, *total_cloud);
    }
}

int main(int argc, char *argv[]) {

	std::string file_path("/mnt/container-data/katori_0906/ply_data/after/enable_rotation/for_calc_volume/TS_1_2/");
    std::string file_code("");
	std::string matrix_path("/mnt/container-data/katori_0906/ply_data/after/enable_rotation/for_regi/TS_1_2/");

    addCloud::SumCloud *sum;
    sum = new addCloud::SumCloud(file_path, matrix_path, file_code);
    sum->make_total_cloud();

    delete sum; 
   
    return 0;
}
