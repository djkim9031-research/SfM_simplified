#include "essential_matrix.h"

void calcEssentialMatrix(const Eigen::Matrix3d &K, const Eigen::Matrix3d &F, Eigen::Matrix3d &E){

    E = (K.transpose())*(F*K);
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d VT = svd.matrixV().transpose();

    //Enforcing rank=2
    Eigen::Vector3d diagElements(1.0, 1.0, 0.0);
    Eigen::Matrix3d S = diagElements.asDiagonal();

    E = U*(S*VT);
}