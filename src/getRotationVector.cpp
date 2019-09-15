#include <fstream>
#include "./../include/getRotationVector.hpp"

GetRotationVector::GetRotationVector() : 
    R (Eigen::Matrix4f::Identity())
{
}

GetRotationVector::~GetRotationVector()
{
}

void GetRotationVector::getRotation(const std::string& filename)
{
    std::vector<float> rotation_vec(16, 0.0);
    std::ifstream ifs(filename);

    for(int i=0; i<16; i++){
        std::string param;
        std::getline(ifs, param);

        rotation_vec[i] = std::stof(param);
    }
            
    /**
     * | vec[0] vec[4] vec[8]  vec[12] |
     * | vec[1] vec[5] vec[9]  vec[13] |
     * | vec[2] vec[6] vec[10] vec[14] |
     * |    0      0      0       1    |
    **/
    R(0,0) = rotation_vec[0];
    R(1,0) = rotation_vec[1];
    R(2,0) = rotation_vec[2];
    R(3,0) = 0.0;
 
    R(0,1) = rotation_vec[4];
    R(1,1) = rotation_vec[5];
    R(2,1) = rotation_vec[6];
    R(3,1) = 0.0;
 
    R(0,2) = rotation_vec[8];
    R(1,2) = rotation_vec[9];
    R(2,2) = rotation_vec[10];
    R(3,2) = 0.0;

    R(0,3) = rotation_vec[12]; 
    R(1,3) = rotation_vec[13]; 
    R(2,3) = rotation_vec[14];
    R(3,3) = 1.0;
}

void GetRotationVector::get_matrix4x4(const std::string &path)
{
    getRotation(path);
}

void GetRotationVector::transformPointCloud(int number) {

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

    std::string filename = "/mnt/container-data/rotation/" + std::to_string(number) + ".txt";
    getRotation(filename);

}
