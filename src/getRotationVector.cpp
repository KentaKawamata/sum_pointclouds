#include <fstream>
#include "./../include/getRotationVector.hpp"

GetRotationVector::GetRotationVector() : 
    R (Eigen::Matrix4f::Identity())
{
}

GetRotationVector::~GetRotationVector() {
    
}

void GetRotationVector::getRotation(std::string& filename){

    std::vector<float> rotation_vec(16, 0.0);
    std::ifstream ifs(filename);

    for(int i=0; i<16; i++){
        std::string param;
        std::getline(ifs, param);

        rotation_vec[i] = std::stof(param);
    }

    R(0,0) = rotation_vec[0];
    R(1,0) = rotation_vec[1];
    R(2,0) = rotation_vec[2];
    R(3,0) = rotation_vec[3];
 
    R(0,1) = rotation_vec[4];
    R(1,1) = rotation_vec[5];
    R(2,1) = rotation_vec[6];
    R(3,1) = rotation_vec[7];
 
    R(0,2) = rotation_vec[8];
    R(1,2) = rotation_vec[9];
    R(2,2) = rotation_vec[10];
    R(3,2) = rotation_vec[11];

    R(0,3) = rotation_vec[12]; 
    R(1,3) = rotation_vec[13]; 
    R(2,3) = rotation_vec[14];
    R(3,3) = rotation_vec[15];
}

void GetRotationVector::transformPointCloud(int number) {

    std::string filename = "/mnt/container-data/rotation/" + std::to_string(number) + ".txt";
    getRotation(filename);

}
