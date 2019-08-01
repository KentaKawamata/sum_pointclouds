#include <iostream>
#include <cmath>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include "./../include/addCloud.hpp"

namespace addCloud {

    SumCloud::SumCloud() : 
        sum_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        filepath ("/mnt/container-data/ply_data/"),
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
        sor.setInputCloud(cloud);
        sor.setLeafSize(voxel_size, voxel_size, voxel_size);
        sor.filter(*cloud);
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
            
            /**
             * getRotationVectorへ数字を渡し,
             * getRotationVectorで得た回転ベクトルRを受け取る
             * 
             * | R00 R01 R02 t0 |
             * | R10 R11 R12 t1 |
             * | R20 R21 R22 t2 |
             * |  0   0   0   1 |
             *
            **/
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

    void SumCloud::getfileNum() {

        namespace fs = boost::filesystem;
        if(!fs::exists(filepath) || !fs::is_directory(filepath)) {
            std::cout << "file path misstake !!" << std::endl;
            exit(0);
        }

        fs::directory_iterator last;
        for(fs::directory_iterator pos(filepath); pos!=last; ++pos) {
            ++count;
        }

        std::cout << "files num : " << std::to_string(count) << std::endl;
    }

    void SumCloud::run() {

        getfileNum();
        addPointCloud();
        savePointcloud();
    }
}

int main(int argc, char *argv[]) {

    addCloud::SumCloud *sum;
    sum = new addCloud::SumCloud();
    sum->run();

    delete sum; 
   
    return 0;
}
