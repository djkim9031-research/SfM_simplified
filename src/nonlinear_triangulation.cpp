#include "nonlinear_triangulation.h"
#include "projection_utils.h"

struct ReprojectionLoss{

    ReprojectionLoss(const Eigen::Vector2d &pt1, 
                     const Eigen::Vector2d &pt2, 
                     const Eigen::Matrix<double, 3, 4> &P1, 
                     const Eigen::Matrix<double, 3, 4> &P2)
    : pt1_(pt1), pt2_(pt2), P1_(P1), P2_(P2) {};

    template<typename T>
    bool operator()(const T* const pt3D_H_vectorized, T* residuals) const {
        // pt3D, one 3D point in homogeneous coord, define a matrix of type T for casting to Ceres Jet type
        Eigen::Matrix<T, 4, 1> pt3D_hom(pt3D_H_vectorized[0], pt3D_H_vectorized[1], pt3D_H_vectorized[2], pt3D_H_vectorized[3]);

        //Project 3D point to image planes;
        Eigen::Matrix<T, 3, 1> projected_pt1 = P1_.cast<T>() * pt3D_hom;
        Eigen::Matrix<T, 3, 1> projected_pt2 = P2_.cast<T>() * pt3D_hom;

        //Normalize
        projected_pt1 /= projected_pt1(2);
        projected_pt2 /= projected_pt2(2);

        // reprojection error for the reference camera point (1)
        residuals[0] = T(pt1_(0)) - projected_pt1(0);
        residuals[1] = T(pt1_(1)) - projected_pt1(1);

        // reprojection error for the second camera point (2)
        residuals[2] = T(pt2_(0)) - projected_pt2(0);
        residuals[3] = T(pt2_(1)) - projected_pt2(1);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d &pt1, const Eigen::Vector2d &pt2, const Eigen::Matrix<double, 3, 4> &P1, const Eigen::Matrix<double, 3, 4> &P2){
        return new ceres::AutoDiffCostFunction<ReprojectionLoss, 4, 4>(new ReprojectionLoss(pt1, pt2, P1, P2));
    }

    Eigen::Vector2d pt1_, pt2_;
    Eigen::Matrix<double, 3, 4> P1_, P2_;
};

void nonLinearTriangulation(const Eigen::Matrix3d &K, 
                            Eigen::MatrixXd &pt1, 
                            Eigen::MatrixXd &pt2, 
                            const Eigen::MatrixXd &pts3D_H, 
                            const Eigen::Matrix3d &R1, 
                            const Eigen::Vector3d &t1, 
                            const Eigen::Matrix3d &R2, 
                            const Eigen::Vector3d &t2, 
                            Eigen::MatrixXd &pts3D_refined_H){

    Eigen::Matrix<double, 3, 4> P1, P2;
    projectionMatrix(R1, t1, K, P1);
    projectionMatrix(R2, t2, K, P2);
    std::vector<Eigen::Vector4d> pts3D_H_vectorized;
    assert(pt1.rows()==pt2.rows());
    assert(pt1.rows()==pts3D_H.rows());

    //Define vector container separately because calculating gradients directly using MatrixXd leads to exploosive gradients
    for(int i=0; i<pts3D_H.rows(); ++i){
        pts3D_H_vectorized.push_back(pts3D_H.row(i).transpose());
    }
    pts3D_refined_H = pts3D_H;

    for(int i=0; i<pts3D_H_vectorized.size(); ++i){

        //set up the problem
        ceres::Problem problem;
        ceres::CostFunction* cost_function = ReprojectionLoss::Create(pt1.row(i).transpose(), pt2.row(i).transpose(), P1, P2);
        problem.AddResidualBlock(cost_function, nullptr, pts3D_H_vectorized[i].data());

        //configure the solver
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_type = ceres::TRUST_REGION;
        options.minimizer_progress_to_stdout = false;

        //run the solver
        ceres::Solver::Summary summary;
        //pts3D_H_vectorized is optimized in place.
        ceres::Solve(options, &problem, &summary);
        pts3D_refined_H.row(i) = pts3D_H_vectorized[i];
        
        //std::cout << summary.FullReport() << std::endl;
    }
    

    return;
}