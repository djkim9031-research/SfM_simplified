#include "projection_utils.h"

void projectionMatrix(const Eigen::Matrix3d &R, 
                      const Eigen::Vector3d &t, 
                      const Eigen::Matrix3d &K, 
                      Eigen::Matrix<double, 3, 4> &P){
    
    Eigen::Matrix<double, 3, 4> trans;
    trans << Eigen::Matrix3d::Identity(), -t;
    P = K*(R*trans);
};

double reprojectionError(const Eigen::MatrixXd &pts3D_H, 
                         const Eigen::MatrixXd &pts2D_1, 
                         const Eigen::MatrixXd &pts2D_2, 
                         const Eigen::Matrix3d &R1, 
                         const Eigen::Vector3d &t1, 
                         const Eigen::Matrix3d &R2, 
                         const Eigen::Vector3d &t2, 
                         const Eigen::Matrix3d &K){

    Eigen::Matrix<double, 3, 4> P1, P2;
    projectionMatrix(R1, t1, K, P1);
    projectionMatrix(R2, t2, K, P2);

    Eigen::MatrixXd uv1 = P1*(pts3D_H.transpose());
    Eigen::MatrixXd scales1 = uv1.row(2).replicate(2, 1);
    Eigen::MatrixXd p1 = (uv1.topRows(2).array()/scales1.array()).transpose();
    
    Eigen::MatrixXd uv2 = P2*(pts3D_H.transpose());
    Eigen::MatrixXd scales2 = uv2.row(2).replicate(2, 1);
    Eigen::MatrixXd p2 = (uv2.topRows(2).array()/scales2.array()).transpose();

    // Calculating Frobenius norms of p1 and p2
    // Frob_norm = [Sigma(over i)[(pt - p)_i]^2]^0.5
    Eigen::MatrixXd norm_p1 = ((pts2D_1 - p1).array().square()).colwise().sum().array().sqrt();
    Eigen::MatrixXd norm_p2 = ((pts2D_2 - p2).array().square()).colwise().sum().array().sqrt();
    
    double e1 = norm_p1.array().square().rowwise().sum().value();
    double e2 = norm_p2.array().square().rowwise().sum().value();
    double num_pts = double(pts3D_H.rows());

    return (e1+e2)/num_pts;
}