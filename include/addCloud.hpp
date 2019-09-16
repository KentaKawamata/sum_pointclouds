#ifndef ADD_CLOUD_H
#define ADD_CLOUD_H

#include <iostream>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/LU>

#include "./getRotationVector.hpp"

namespace addCloud {

    class SumCloud {

    private:

        int count;
        std::string filepath;
        std::string matrixpath;
        std::string filename;
        std::string file_code;
        
        Eigen::Matrix4f R;

        pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud;

        void addPointCloud();
        void savePointcloud();
        int getfileNum();
        void outlineFilter();
        void voxelization_filter();
        void filteredCloud();

    public:

        SumCloud(const std::string &file_path, 
                 const std::string &matrix_path, 
                 const std::string &code);
        ~SumCloud();
        void run();
        void make_total_cloud();

        GetRotationVector *rote;
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //ADD_CLOUD_H