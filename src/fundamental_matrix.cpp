#include "fundamental_matrix.h"

void normalize(const Eigen::MatrixXd &pts, Eigen::MatrixXd &x_norm, Eigen::MatrixXd &T){
    //https://sites.cc.gatech.edu/classes/AY2016/cs4476_fall/results/proj3/html/sdai30/index.html

    Eigen::RowVector2d uv_ = pts.colwise().mean();
    Eigen::MatrixXd uv_cap = (pts - pts.colwise().mean().replicate(8, 1)).array();
    double u_ = uv_(0);
    double v_ = uv_(1);
    Eigen::VectorXd u_cap = uv_cap.col(0);
    Eigen::VectorXd v_cap = uv_cap.col(1);

    //Scale term s to be the average distance of the centered points from the origin
    //s = sqrt(2)/sqrt(mean_of_all(u^2 + v^2))
    double s = (2.0/(u_cap.array().square() + v_cap.array().square()).colwise().mean()).sqrt().value();
    
    //translate by the mean centroid, then scale by s
    Eigen::Vector3d diagElements(s, s, 1.0);
    Eigen::Matrix3d T_scale = diagElements.asDiagonal();
    Eigen::Matrix3d T_translation;
    T_translation << 1.0, 0.0, -u_,
                     0.0, 1.0, -v_,
                     0.0, 0.0, 1.0;
    T = T_scale*T_translation;

    Eigen::MatrixXd x(8,3);
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(8);
    x.leftCols(2) = pts;
    x.rightCols(1) = ones;
    x_norm = (T*(x.transpose())).transpose();
}

void estimateFundamentalMatrix(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2, Eigen::Matrix3d &F){
    assert(pts1.rows() == pts2.rows());
    assert(pts1.rows() >= 8);
    
    Eigen::MatrixXd x1_norm, x2_norm, T1, T2;
    normalize(pts1, x1_norm, T1);
    normalize(pts2, x2_norm, T2);

    // 8-point algorithm for fundamental matrix estimation.
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(8, 9);
    for(int i=0; i<8; ++i){
        double x_1, y_1, x_2, y_2;
        x_1 = x1_norm(i, 0);
        y_1 = x1_norm(i, 1);
        x_2 = x2_norm(i, 0);
        y_2 = x2_norm(i, 1);

        Eigen::RowVectorXd values(9);
        values<< x_1*x_2, x_2*y_1, x_2, y_2*x_1, y_2*y_1, y_2, x_1, y_1, 1.0;
        A.row(i) = values;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    // take last column of V corresponding to the least singular value
    Eigen::Matrix3d temp_F;
    for(int i=0; i<V.rows(); ++i){
        temp_F(i/3, i%3) = V(i, V.cols()-1);
    }

    // Enforce rank 2 for F
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_F(temp_F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d diagElements(svd_F.singularValues()(0), svd_F.singularValues()(1), 0.0);
    Eigen::Matrix3d enforced_S = diagElements.asDiagonal();

    F = svd_F.matrixU()*enforced_S*(svd_F.matrixV().transpose());

    //de-normalization
    F = T2.transpose()*(F*T1);
    double scale = F(2,2);
    F = F.array()/scale;
}