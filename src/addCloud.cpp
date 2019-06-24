#include <iostream>
#include <boost/filesystem.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "./../include/addCloud.hpp"

namespace addCloud {

    SumCloud::SumCloud() : 
        sum_cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        cloud (new pcl::PointCloud<pcl::PointXYZ>()),
        filepath ("/mnt/datas/ply_data/"),
        count (0)
    {
    }

    SumCloud::~SumCloud() {
    }

    void SumCloud::addPointCloud() {

        for(int i=0; i<count; i++) {

            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

            filename = filepath + std::to_string(i) + ".ply";
            pcl::io::loadPLYFile(filename, *cloud);
        
            *sum_cloud += *cloud;
        }
    }

    void SumCloud::savePointcloud() {

        std::string save_filename = "/root/ply_data/test.ply"; 
        std::cout << "SAVE FILE NAME : " << save_filename << std::endl;
        pcl::io::savePLYFileASCII(filename, *sum_cloud);
    }

    void SumCloud::getfileNum() {

        namespace fs = boost::filesystem;
        if(!fs::exists(filepath) || !fs::is_directory(filepath)) {
            exit(0);
        }

        fs::directory_iterator last;
        for(fs::directory_iterator pos(filepath); pos!=last; ++pos) {
            ++count;
        }
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
