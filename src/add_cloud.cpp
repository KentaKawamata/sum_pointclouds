#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelization_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>() );
    float voxel_size = 0.1;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*filtered_cloud);

    //std::cout << "cloud num : " << cloud->size() << std::endl;

    return filtered_cloud;
}


void add_cloud(const std::string &file_path,
               const std::string &file_name,
               const int num)
{
    std::string format = ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_cloud (new pcl::PointCloud<pcl::PointXYZ>() );

    for(int i=0; i<num; i++)
    {
        std::string cloud_name = file_path 
                               + file_name 
                               + std::to_string(i) 
                               + format;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>() );
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered (new pcl::PointCloud<pcl::PointXYZ>() );

        pcl::io::loadPCDFile(cloud_name, *cloud);
        filtered = voxelization_filter(cloud);

        std::string cloud_save = file_path + file_name + "_filtered" + std::to_string(i) + format;
        pcl::io::savePCDFileASCII(cloud_save, *filtered);

        *total_cloud += *filtered;
        
        std::cout << "add cloud : " << file_name + std::to_string(i) + format << std::endl; 
    }

    std::cout << "cloud num : " << total_cloud->size() << std::endl;

    std::string save_path = "/mnt/container-data/filterreg_data/";
    std::string save_name = "katori_after";
    std::string cloud_save = save_path + save_name + format;

    pcl::io::savePCDFileASCII(cloud_save, *total_cloud);
}


int main(int argc, char *argv[])
{
    if(argc != 2)
    {
        std::cout << "Please input file num .add_cloud <file num>" << std::endl;
        std::exit(1);
    }

    std::string file_path = "/mnt/container-data/filterreg_data/katori_transform/";
    std::string file_name = "cloud_";
    int file_num = std::atoi(argv[1]);

    add_cloud(file_path, file_name, file_num);

    return 0;
}
