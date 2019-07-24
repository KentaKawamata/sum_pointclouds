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
        std::string filename;
        
        Eigen::Matrix4d R;

        pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        void addPointCloud();
        void savePointcloud();
        void getfileNum();
        void outlineFilter();
        void voxelization_filter();

    public:

        SumCloud();
        ~SumCloud();
        void run();

        GetRotationVector *rote;
    
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif //ADD_CLOUD_H