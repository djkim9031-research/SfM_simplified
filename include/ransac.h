#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cassert>
#include <random>
#include <set>

// Function overloads for 3x3 matrix estimation given a pair of 2D matched points.
// Currently only supports to estimate a fundamental matrix, but this pipeline can be
// extended to estimate an essential matrix, and/or a homogragraphy matrix.
//
// @param pts2D_1           Matched pixels of view1.
// @param pts2D_2           Matched pixels of view2.
// @param idxs              Correspondence indices read from the matching.txt data
//                          It represents the i-th row at which a 2D matched correspondence 
//                          between view1 and view2 exists.
// 
// @param Estimated_matrix  Estimated 3x3 matrix.
// @param final_idxs        Filtered idxs, which were used to obtain the best 3x3 matrix estimation.
void RANSAC(const Eigen::MatrixXd &pts2D_1, 
            const Eigen::MatrixXd &pts2D_2, 
            const std::vector<int> &idxs, 
            Eigen::Matrix3d &Estimated_matrix, 
            std::vector<int> &final_idxs);

// Function overloads for PnP pose estimation given a set of 2D points, a set of 3D points,
// and the corresponding camera intrinsic matrix.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param pts2D             2D points captured from the given camera.
// @param pts3D             3D points captured from the given camera.
// 
// @param Estimated_R       Estimated 3x3 rotation matrix of the camera.
// @param Estimated_t       Estimated 3x1 translation vector of the camera.
void RANSAC(const Eigen::Matrix3d &K, 
            const Eigen::MatrixXd &pts2D, 
            const Eigen::MatrixXd &pts3D, 
            Eigen::Matrix3d &Estimated_R, 
            Eigen::Vector3d &Estimated_t);