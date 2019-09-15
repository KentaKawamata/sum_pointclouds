#ifndef GET_ROTATION_VECTOR_H
#define GET_ROTATION_VECTOR_H

#include <Eigen/Core>
#include <Eigen/LU>

class GetRotationVector {

private:

    void getRotation(const std::string& filename);

public:

    Eigen::Matrix4f R;

    GetRotationVector();
    ~GetRotationVector();
    void transformPointCloud(int number);
    void get_matrix4x4(const std::string& filename);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //GET_ROTATION_VECTOR_H