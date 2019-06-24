#ifndef ADD_CLOUD_H
#define ADD_CLOUD_H

#include <iostream>
#include <pcl/point_types.h>

namespace addCloud {

    class SumCloud {

    private:

        int count;
        std::string filepath;
        std::string filename;

        pcl::PointCloud<pcl::PointXYZ>::Ptr sum_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        void addPointCloud();
        void savePointcloud();
        void getfileNum();

    public:

        SumCloud();
        ~SumCloud();
        void run();
    };
}

#endif //ADD_CLOUD_H