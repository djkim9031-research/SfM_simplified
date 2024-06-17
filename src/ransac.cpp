#include "ransac.h"
#include "fundamental_matrix.h"
#include "algorithms_PnP.h"

namespace{

    void random_selection(const int num_to_select,
                          const Eigen::MatrixXd& matrix1,
                          const Eigen::MatrixXd& matrix2,
                          Eigen::MatrixXd& selected_matrix1,
                          Eigen::MatrixXd& selected_matrix2){
        
        //Randomly select `num_to_select` rows
        int n_rows = matrix1.rows();
        std::set<int> rnd_indices;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0, n_rows - 1);

        while(rnd_indices.size() < num_to_select){
            int randIndex = distrib(gen);
            rnd_indices.insert(randIndex);
        }

        Eigen::MatrixXd selection1(num_to_select, matrix1.cols());
        Eigen::MatrixXd selection2(num_to_select, matrix2.cols());
        int rowIndex = 0;
        for(int index : rnd_indices){
            selection1.row(rowIndex) = matrix1.row(index);
            selection2.row(rowIndex) = matrix2.row(index);
            rowIndex++;
        }

        selected_matrix1 = selection1;
        selected_matrix2 = selection2;
        return;
    }

    double epipolarConstraintError(const Eigen::RowVector2d &pts1, 
                                   const Eigen::RowVector2d &pts2, 
                                   const Eigen::Matrix3d &Matrix){
        // Error = X'.T*F*X, should ideally be zero as per epipolar constraint
        Eigen::MatrixXd pt1(1, 3);
        Eigen::MatrixXd pt2(1, 3);
        Eigen::VectorXd ones = Eigen::VectorXd::Ones(1);
        pt1.leftCols(2) = pts1;
        pt1.rightCols(1) = ones;
        pt2.leftCols(2) = pts2;
        pt2.rightCols(1) = ones;

        return (pt2*(Matrix*pt1.transpose())).array().abs().value();
    }

} // end of namespace

// RANSAC for 3x3 matrix estimation
void RANSAC(const Eigen::MatrixXd &pts2D_1, 
            const Eigen::MatrixXd &pts2D_2, 
            const std::vector<int> &idxs, 
            Eigen::Matrix3d &Estimated_matrix, 
            std::vector<int> &final_idxs){
    
    assert(pts2D_1.rows() == pts2D_2.rows());
    int n_rows = pts2D_1.rows();

    const int n_iterations = 2000;
    const float error_threshold = 0.002;
    int inliers_best = 0;

    for(int i=0; i<n_iterations; ++i){
        //Randomly select 8 rows
        Eigen::MatrixXd pts1_8(8, pts2D_1.cols());
        Eigen::MatrixXd pts2_8(8, pts2D_2.cols());
        random_selection(8, pts2D_1, pts2D_2, pts1_8, pts2_8);

        Eigen::Matrix3d f_8;
        estimateFundamentalMatrix(pts1_8, pts2_8, f_8);
        std::vector<int> indices;
        for(int j=0; j<n_rows; ++j){
            double error = epipolarConstraintError(pts2D_1.row(j), pts2D_2.row(j), f_8);
            if(error<error_threshold){
                indices.push_back(idxs[j]);
            }
        }
        if(indices.size() > inliers_best){
            inliers_best = indices.size();
            final_idxs = indices;
            Estimated_matrix = f_8;
        }
    }
}

// RANSAC for PnP pose estimation
void RANSAC(const Eigen::Matrix3d &K, 
            const Eigen::MatrixXd &pts2D, 
            const Eigen::MatrixXd &pts3D, 
            Eigen::Matrix3d &Estimated_R, 
            Eigen::Vector3d &Estimated_t){
    
    assert(pts2D.rows() == pts3D.rows());
    int n_rows = pts2D.rows();

    const int n_iterations = 1000;
    const float error_threshold = 10.0;
    int inliers_thresh = 0;
    for(int i=0;i<n_iterations;++i){
        //Randomly select 6 rows
        Eigen::MatrixXd selected_pts2D(6, pts2D.cols());
        Eigen::MatrixXd selected_pts3D(6, pts3D.cols());
        random_selection(8, pts2D, pts3D, selected_pts2D, selected_pts3D);
        
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        linearPnP(K, selected_pts2D, selected_pts3D, R, t);

        std::vector<int> indices;
        for(int j=0;j<n_rows;++j){
            Eigen::MatrixXd pt2D(1, 2);
            Eigen::MatrixXd pt3D(1, 3);
            pt2D = pts2D.block(j, 0, 1, 2);
            pt3D = pts3D.block(j, 0, 1, 3);
            double error = PnPError(K, pt2D, pt3D, R, t);
            if(error<error_threshold){
                indices.push_back(j);
            }
        }
        if(indices.size() > inliers_thresh){
            inliers_thresh = indices.size();
            Estimated_R = R;
            Estimated_t = t;
        }
    }
}