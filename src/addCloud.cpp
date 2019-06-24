#include <iostream>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/addCloud.hpp"

namespace addCloud {

    SumCloud::SumCloud() : 
        sum_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        filepath ("/mnt/container-data/ply_data/"),
        count (0)
    {
    }

    SumCloud::~SumCloud() {
    }

    void SumCloud::addPointCloud() {

        for(int i=0; i<count; i++) {

            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

            filename = filepath + std::to_string(i) + ".ply";
            std::cout << "READ FILE NAME : " << filename << std::endl;
            pcl::io::loadPLYFile(filename, *cloud);
        
            *sum_cloud += *cloud;
        }
    }

    void SumCloud::savePointcloud() {

        std::string save_filename = "/mnt/container-data/test.ply"; 
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
