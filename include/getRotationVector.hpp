#ifndef GET_ROTATION_VECTOR_H
#define GET_ROTATION_VECTOR_H

#include <Eigen/Core>
#include <Eigen/LU>

class GetRotationVector {

private:

    void getRotation(std::string& filename);

public:

    Eigen::Matrix4d R;

    GetRotationVector();
    ~GetRotationVector();
    void transformPointCloud(int number);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif //GET_ROTATION_VECTOR_H