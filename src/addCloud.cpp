#include <iostream>
#include <cmath>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "./../include/addCloud.hpp"

namespace addCloud {

    SumCloud::SumCloud(const std::string &file_path) : 
        sum_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        total_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        filepath (file_path),
        R (Eigen::Matrix4f::Identity()),
        count (0)
    {
        rote = new GetRotationVector();
    }

    SumCloud::~SumCloud() {
        delete rote;
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

    void SumCloud::filteredCloud(){
    
        pcl::PassThrough<pcl::PointXYZ> passX;
        passX.setInputCloud(cloud);
        passX.setFilterFieldName("x");
        passX.setFilterLimits(-3.2, 3.2);
        passX.setFilterLimitsNegative (false);
        passX.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passY;
        passY.setInputCloud(cloud);
        passY.setFilterFieldName("y");
        passY.setFilterLimits(0.0, 2.6);
        passY.setFilterLimitsNegative (false);
        passY.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> passZ;
        passZ.setInputCloud(cloud);
        passZ.setFilterFieldName("z");
        passZ.setFilterLimits(0.0, 4.0);
        passZ.setFilterLimitsNegative (false);
        passZ.filter(*cloud);
    }


    void SumCloud::addPointCloud() {

        for(int i=0; i<count; i++) {

            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
            //tmp_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);


            filename = filepath + std::to_string(i) + ".ply";
            std::cout << "READ FILE NAME : " << filename << std::endl;
            pcl::io::loadPLYFile(filename, *cloud);

            filteredCloud();
            
            rote->transformPointCloud(i);
            R = rote->R;
            
            pcl::transformPointCloud(*cloud, *cloud, R);

            voxelization_filter();
            outlineFilter();
        
            *sum_cloud += *cloud;

        }
    }

    void SumCloud::savePointcloud() {

        std::string save_filename = "/mnt/container-data/test_0724.ply"; 
        std::cout << "SAVE FILE NAME : " << save_filename << std::endl;
        pcl::io::savePLYFileASCII(save_filename, *sum_cloud);
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

    void SumCloud::run() {

        int i = getfileNum();
        addPointCloud();
        savePointcloud();
    }

    void SumCloud::make_total_cloud()
    {
        int num = getfileNum();

        for(int i=0; i<num; i++)
        {
            std::string cloud_name = filepath + std::to_string(i) + ".ply"; 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPLYFile(cloud_name, *cloud);

            for(int j=i; j>0; j--)
            {
	            std::string matrix_name = filepath 
                                        + "matrix" 
                                        + std::to_string(j-1) 
                                        + "-" 
                                        + std::to_string(j) 
                                        + ".txt";

                rote->get_matrix4x4(matrix_name);
                R = rote->R;
                pcl::transformPointCloud(*cloud, *cloud, R);
            }
            outlineFilter();
            *total_cloud += *cloud; 
            voxelization_filter();
        }
         
        std::string save_name = "/mnt/container-data/katori_0906/ply_total/after1-2.ply";
        pcl::io::savePLYFileASCII(save_name, *total_cloud);
    }
}

int main(int argc, char *argv[]) {

	std::string file_path("/mnt/container-data/katori_0906/ply_data/after/enable_rotation/for/regi/TS_1-2/");
	int num;
	try
	{
		num = std::stoi(argv[1]) - 1;
	}
	catch(const std::exception& e)
	{
		//std::cerr << e.what() << '\n';
	}
	// model       : observationに対して位置を合わせる点群 
	// observation : 基準となる点群
	std::string model_path = file_path + argv[1] + ".ply";
	std::string obs_path = file_path + std::to_string(num) + ".ply";

    addCloud::SumCloud *sum;
    sum = new addCloud::SumCloud(file_path);
    sum->make_total_cloud();

    delete sum; 
   
    return 0;
}
