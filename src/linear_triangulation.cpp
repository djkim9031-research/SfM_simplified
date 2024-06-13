#include "linear_triangulation.h"

void linearTriangulation(const Eigen::Matrix3d &K, 
                         const Eigen::Matrix3d &R1,
                         const Eigen::Vector3d &t1,
                         const Eigen::Matrix3d &R2, 
                         const Eigen::Vector3d &t2, 
                         const Eigen::MatrixXd &pts2D_1, 
                         const Eigen::MatrixXd &pts2D_2, 
                         Eigen::MatrixXd &pts3D_H){

    Eigen::Matrix<double, 3, 4> trans1;
    Eigen::Matrix<double, 3, 4> trans2;
    trans1 << Eigen::Matrix3d::Identity(), -t1;
    trans2 << Eigen::Matrix3d::Identity(), -t2;

    // Perform the multiplication R * [I | -B]
    Eigen::Matrix<double, 3, 4> motion1 = R1 * trans1;
    Eigen::Matrix<double, 3, 4> motion2 = R2 * trans2;

    // Projection matrix
    Eigen::Matrix<double, 3, 4> P1 = K * motion1;
    Eigen::Matrix<double, 3, 4> P2 = K * motion2;

    Eigen::Matrix<double, 1, 4> p1T = P1.row(0);
    Eigen::Matrix<double, 1, 4> p2T = P1.row(1);
    Eigen::Matrix<double, 1, 4> p3T = P1.row(2);

    Eigen::Matrix<double, 1, 4> p_dash_1T = P2.row(0);
    Eigen::Matrix<double, 1, 4> p_dash_2T = P2.row(1);
    Eigen::Matrix<double, 1, 4> p_dash_3T = P2.row(2);

    int num_pts = pts2D_1.rows();
    Eigen::MatrixXd X_all = Eigen::MatrixXd::Zero(num_pts, 4);
    for(unsigned int i=0;i<num_pts;++i){
        double x = pts2D_1(i, 0);
        double y = pts2D_1(i, 1);
        double x_dash = pts2D_2(i, 0);
        double y_dash = pts2D_2(i, 1);

        // Constructing a system of equations - removing the homogenous scale factor
        // The pixel coordinate to normalized image coordinate =>
        // (u, v, w).T = P*X, and x*(p3T * X) = u = p1T*X// y*(p3T*X) = v = p2T*X
        // Hence, x*p3T = p1T, y*p3T = p2T, form the matrix A for AX = 0
        Eigen::Matrix<double, 1, 4> eq1, eq2, eq3, eq4;
        eq1 = (x*p3T - p1T);
        eq2 = (y*p3T - p2T);
        eq3 = (x_dash*p_dash_3T - p_dash_1T);
        eq4 = (y_dash*p_dash_3T - p_dash_2T);

        Eigen::Matrix<double, 4, 4> A;
        A << eq1,
             eq2,
             eq3,
             eq4;
        
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix4d V = svd.matrixV();
        Eigen::Matrix<double, 4, 1> X = V.col(3);

        // Deriving x,y,z values in homogenous form for this point index
        // Divide by X(3,0) to normalize, and manually set the 4th row 
        // to 1 to enforce the homogeneous coordinate.
        X_all(i, 0) = X(0, 0)/X(3, 0);
        X_all(i, 1) = X(1, 0)/X(3, 0);
        X_all(i, 2) = X(2, 0)/X(3, 0);
        X_all(i, 3) = 1.0;
    }
    pts3D_H = X_all;

    return;
}