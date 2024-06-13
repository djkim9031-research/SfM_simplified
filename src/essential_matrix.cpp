#include "essential_matrix.h"

#include <limits>

namespace {
    // Helper function to check point's Z value in respective camera coordinate (with estimated R, t).
    // Count the number of positive Z values in the camera coordinate and returns the number.
    //
    // @param pts3D             3D points (inhomogeneous coord) for the particular R and t.
    // @param R_r3              3rd row of the rotation matrix (responsible for Z value transformation)
    // @param t                 Translation vector.
    //
    // @return                  Total number of 3D points with positive Z under current R and t.
    int depthPositiveConstraint(const Eigen::MatrixXd &pts3D, const Eigen::Matrix<double, 1, 3> &R_r3, const Eigen::Matrix<double, 3, 1> &t){
        int n_positive_z = 0;
        for(unsigned int i=0; i<pts3D.rows(); ++i){
            Eigen::Matrix<double, 3, 1> curr_pt3D;
            curr_pt3D << pts3D.row(i).transpose();
            
            // z1 (for estimating positive z in world coord = first camera pose)
            // z2 (for estimating positive z in cam coord = current camera pose)
            double z1 = curr_pt3D(2, 0);
            double z2 = R_r3*(curr_pt3D - t);
            if(z1>0 && z2>0){
                n_positive_z++;
            }
        }
        return n_positive_z;
    }
} // end of namespace

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

void extractCameraPose(const Eigen::Matrix3d &E, std::vector<Eigen::Matrix3d> &R, std::vector<Eigen::Vector3d> &t){
    // E = U*diag(1,1,0)*V.T
    // and ZW = diag(1,1,0) where Z = [0 1 0 // -1 0 0 // 0 0 0]: skew symmetric matrix
    // W = [0 -1 0// 1 0 0// 0 0 1]: rotation matrix
    // E = U*Z*W*V.T = (U*Z*U.T)*(U*W*V.T) = (Baseline)*(Rotation)
    // 4 possibilities: ZW, -Z.T*W, -Z*W.T, Z.T*W.T = diag(1,1,0)
    // Also, from SVD, the 3rd columns of U and V correspond to the null space of E (singular values = 0), 
    // This is the direction of epipole (parallel to the translation vector)

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d VT = svd.matrixV().transpose();
    Eigen::Vector3d S = svd.singularValues();
    Eigen::Matrix3d W;
    W << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    
    R.push_back((U*(W*VT)));
    R.push_back((U*(W*VT)));
    R.push_back((U*(W.transpose()*VT)));
    R.push_back((U*(W.transpose()*VT)));

    t.push_back(U.col(2));
    t.push_back(-U.col(2));
    t.push_back(U.col(2));
    t.push_back(-U.col(2));

    //check the sign of rotation matrix, if negative correct by negation
    for(unsigned int i=0; i<R.size(); ++i){
        if(R[i].determinant() < 0){
            R[i] = -R[i];
            t[i] = -t[i];
        }
    }
    return;
}

void disambiguatePose(const std::vector<Eigen::Matrix3d> &R,
                      const std::vector<Eigen::Vector3d> &t, 
                      const std::vector<Eigen::MatrixXd> &pts3D_H, 
                      Eigen::Matrix3d &best_R, 
                      Eigen::Vector3d &best_t, 
                      Eigen::MatrixXd &best_pts3D_H){
    //Z : r3(X-C) > 0

    unsigned int best_i = 0;
    int max_positive_depth = std::numeric_limits<int>::min();
    int num_pts = pts3D_H[0].rows();
    for(unsigned int i=0;i<R.size(); ++i){
        Eigen::Matrix3d curr_R = R[i];
        Eigen::Matrix<double, 3, 1> curr_B;
        curr_B << t[i];

        Eigen::Matrix<double, 1, 3> r3 = curr_R.row(2);
        Eigen::MatrixXd pts3D = pts3D_H[i].block(0, 0, num_pts, 3);
        int n_positive_depth = depthPositiveConstraint(pts3D, r3, curr_B);
        if(n_positive_depth > max_positive_depth){
            max_positive_depth = n_positive_depth;
            best_i = i;
        }
    }

    best_R = R[best_i];
    best_t = t[best_i];
    best_pts3D_H = pts3D_H[best_i];
    return;

}