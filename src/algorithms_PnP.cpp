#include "algorithms_PnP.h"
#include "projection_utils.h"

namespace{

    // Helper function to make homogenoues matrix from a given matrix.
    // It converts M x N matrix to M x (N+1) matrix.
    Eigen::MatrixXd makeHomogeneous(const Eigen::MatrixXd& matrix) {
        // Get the size of the input matrix
        int rows = matrix.rows();
        int cols = matrix.cols();

        // Create a new matrix with one additional row
        Eigen::MatrixXd homogeneousMatrix(rows, cols+1);
        // Copy the original matrix data
        homogeneousMatrix.leftCols(cols) = matrix;
        // Set the last cols to ones
        homogeneousMatrix.col(cols) = Eigen::MatrixXd::Ones(rows, 1);
        
        return homogeneousMatrix;
    }

} // end of namespace

void linearPnP(const Eigen::Matrix3d &K, 
               const Eigen::MatrixXd &pts2D, 
               const Eigen::MatrixXd &pts3D, 
               Eigen::Matrix3d &R, 
               Eigen::Vector3d &t){

    int n = pts2D.rows();
    Eigen::MatrixXd pts2D_H = makeHomogeneous(pts2D);
    Eigen::MatrixXd pts3D_H = makeHomogeneous(pts3D);

    //x = KR[I | -t]X
    //(normalize img coord) K^-1*x = R[I | -t]X
    Eigen::MatrixXd xn = ((K.inverse())*(pts2D_H.transpose())).transpose(); // num_pts, 3
    Eigen::MatrixXd A(3*n, 12);
    for(int i=0; i<n; ++i){
        double u = xn(i, 0);
        double v = xn(i, 1);

        //cross product of xn and PX = 0, since same direction vector where P = R[I|-t]
        //Using skew symmetric matrix of xn, skew(xn)*PX = 0
        //Rearranging to make Aw = 0 form for vectors of P = [P1, P2, P3].T 3 rows of (1x4) vectors, 
        //A = skew(xn)*[X 0 0, 0 X 0, 0 0 X]
        //w = [P1.T, P2.T, P3.T].T of size (12, 1) = 3 rows of (4x1) vectors
        //where X and 0 are 1x4 vectors, so A = 3x12 per point
        //Accuring A over all points to generate 3nx12 matrix
        Eigen::Matrix3d skew_xn(3, 3);
        skew_xn << 0, -1, v,
                   1, 0, -u,
                   -v, u, 0;
        Eigen::MatrixXd X_tilda(3, 12);
        X_tilda << pts3D_H(i, 0), pts3D_H(i, 1), pts3D_H(i, 2), pts3D_H(i, 3), 0, 0, 0, 0, 0, 0, 0, 0,
                   0, 0, 0, 0, pts3D_H(i, 0), pts3D_H(i, 1), pts3D_H(i, 2), pts3D_H(i, 3), 0, 0, 0, 0,
                   0, 0, 0, 0, 0, 0, 0, 0, pts3D_H(i, 0), pts3D_H(i, 1), pts3D_H(i, 2), pts3D_H(i, 3);

        A.block(3*i, 0, 3, 12) = (skew_xn*X_tilda);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV(); //12x12 matrix
    // take last column of V corresponding to the least singular value = P
    Eigen::MatrixXd temp_P(3, 4);
    for(int i=0; i<V.rows(); ++i){
        temp_P(i/4, i%4) = V(i, V.cols()-1);
    }
    Eigen::Matrix3d Rotation = temp_P.block(0, 0, 3, 3);
    Eigen::MatrixXd Center = temp_P.block(0, 3, 3, 1);

    //decompose and enforce orthogonality
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_R(Rotation, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U_R = svd_R.matrixU();
    Eigen::Matrix3d VT_R = svd_R.matrixV().transpose();
    Rotation = U_R*VT_R;

    Eigen::Vector3d translation, translation_;
    translation = -(Rotation.transpose())*Center;
    //For sanity check (orthogonality of Rotation matrix)
    translation_ = -(Rotation.inverse())*Center;
    double tolerance = 1e-5;
    assert(translation.isApprox(translation_, tolerance) && "translation matrix orthogonality is not held");
   
    //Since Rotation is an orthogonal matrix, det() is either +1 or -1
    //Correct the sign if negative
    if(Rotation.determinant()<0){
        R = -Rotation;
        t = -translation;
        return;
    }

    R = Rotation;
    t = translation;
    return;
};

double PnPError(const Eigen::Matrix3d &K,
                const Eigen::MatrixXd &pt2D, 
                const Eigen::MatrixXd &pt3D, 
                const Eigen::Matrix3d &R, 
                const Eigen::Vector3d &B){
    Eigen::MatrixXd pt3D_H = makeHomogeneous(pt3D);

    Eigen::Matrix<double, 3, 4> P;
    projectionMatrix(R, B, K, P);

    double u_proj = ((P.row(0))*pt3D_H.transpose()).value()/((P.row(2))*pt3D_H.transpose()).value();
    double v_proj = ((P.row(1))*pt3D_H.transpose()).value()/((P.row(2))*pt3D_H.transpose()).value();
    Eigen::RowVector2d x_proj(u_proj, v_proj);
    
    double e = (pt2D.row(0) - x_proj).norm();
    //double e = sqrt(pow(u_proj - x(0, 0), 2.0) + pow(v_proj - x(0, 1), 2.0));
    return e;
};

struct PnPLoss{

    PnPLoss(const Eigen::MatrixXd& pts2D, const Eigen::MatrixXd& pts3D, const Eigen::Matrix3d &K)
    : pts2D_(pts2D), pts3D_(pts3D), K_(K) {};

    template<typename T>
    Eigen::Matrix<T, 3, 3> QuaternionToRotationMatrix(const Eigen::Matrix<T, 4, 1>& q){
        //Normalize the quaternion
        T norm = ceres::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        T w = q[0]/norm;
        T x = q[1]/norm;
        T y = q[2]/norm;
        T z = q[3]/norm;

        Eigen::Matrix<T, 3, 3> R;
        R(0, 0) = 1.0 - 2.0 * y * y - 2.0 * z * z;
        R(0, 1) = 2.0 * x * y - 2.0 * w * z;
        R(0, 2) = 2.0 * x * z + 2.0 * w * y;
        R(1, 0) = 2.0 * x * y + 2.0 * w * z;
        R(1, 1) = 1.0 - 2.0 * x * x - 2.0 * z * z;
        R(1, 2) = 2.0 * y * z - 2.0 * w * x;
        R(2, 0) = 2.0 * x * z - 2.0 * w * y;
        R(2, 1) = 2.0 * y * z + 2.0 * w * x;
        R(2, 2) = 1.0 - 2.0 * x * x - 2.0 * y * y;

        return R;
    }

    template<typename T>
    void projectionMatrix(const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &t, 
                          const Eigen::Matrix<T, 3, 3> &K, Eigen::Matrix<T, 3, 4> &P){
        
        Eigen::Matrix<T, 3, 4> trans;
        trans << Eigen::Matrix<T, 3, 3>::Identity(), -t;
        P = K * (R * trans);        
    }

    template<typename T>
    bool operator()(const T* const X0, T* residuals) const {

        Eigen::Quaternion<T> q(X0[0], X0[1], X0[2], X0[3]);
        q.normalize();
        Eigen::Matrix<T, 3, 1> t(X0[4], X0[5], X0[6]);
        Eigen::Matrix<T, 3, 3> R = q.toRotationMatrix();
        Eigen::Matrix<T, 3, 4> P;
        Eigen::Matrix<T, 3, 4> trans;
        trans << Eigen::Matrix<T, 3, 3>::Identity(), -t;
        P = K_.cast<T>() * (R * trans);   

        for(int i=0; i<pts3D_.rows(); ++i){
            Eigen::Matrix<T, 4, 1> pts3D_H_(T(pts3D_(i, 0)), T(pts3D_(i, 1)), T(pts3D_(i, 2)), T(1));
            Eigen::Matrix<T, 3, 1> projected = P*pts3D_H_;
            projected /= projected(2);

            residuals[i*2 + 0] = pts2D_(i, 0) - projected(0);
            residuals[i*2 + 1] = pts2D_(i, 1) - projected(1);
        }

        return true;
    }
    
    static ceres::CostFunction* create(const Eigen::MatrixXd &pts2D, const Eigen::MatrixXd &pts3D, const Eigen::Matrix3d &K){
        return new ceres::AutoDiffCostFunction<PnPLoss, ceres::DYNAMIC, 7>(
            new PnPLoss(pts2D, pts3D, K), pts2D.rows()*2);
    }

    Eigen::MatrixXd pts2D_, pts3D_;
    Eigen::Matrix3d K_;
};

void nonLinearPnP(const Eigen::Matrix3d &K, 
                  const Eigen::MatrixXd &pts2D, 
                  const Eigen::MatrixXd &pts3D, 
                  Eigen::Matrix3d &R, 
                  Eigen::Vector3d &t){
    
    Eigen::Quaterniond q(R);
    double X0[7] = {q.w(), q.x(), q.y(), q.z(), t(0), t(1), t(2)};
    ceres::Problem problem;
    ceres::CostFunction *cost_function = PnPLoss::create(pts2D, pts3D, K);
    problem.AddResidualBlock(cost_function, nullptr, X0);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //std::cout << summary.FullReport() << std::endl;

    Eigen::Quaterniond optimizedQ(X0[0], X0[1], X0[2], X0[3]);
    Eigen::Vector3d optimizedB(X0[4], X0[5], X0[6]);

    optimizedQ.normalize();
    R = optimizedQ.toRotationMatrix();
    t = optimizedB;
}

double reprojectionErrorPnP(const Eigen::Matrix3d &K, 
                            const Eigen::MatrixXd &pts2D, 
                            const Eigen::MatrixXd &pts3D,
                            const Eigen::Matrix3d &R, 
                            const Eigen::Vector3d &t){

    Eigen::Matrix<double, 3, 4> P;
    projectionMatrix(R, t, K, P);
    Eigen::MatrixXd pts3D_H = makeHomogeneous(pts3D);

    Eigen::MatrixXd uv = P*(pts3D_H.transpose()); // 3xnum_pts
    Eigen::MatrixXd scales = uv.row(2).replicate(2, 1);
    Eigen::MatrixXd p1 = (uv.topRows(2).array()/scales.array()).transpose(); //num_ptsx2

    Eigen::MatrixXd norm_p = ((pts2D - p1).array().square()).colwise().sum().array().sqrt();
    double e = norm_p.array().square().rowwise().sum().value();
    return e/double(pts3D.rows());
}