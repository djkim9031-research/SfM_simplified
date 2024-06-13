#pragma once
#include <Eigen/Dense>
#include <vector>

// Calculating the essential matrix from camera calibration matrix and a fundamental matrix.
// This assumes the camera is already calibrated and the intrinsic matrix is known.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param F                 3x3 Fundamental matrix 
//
// @param E                 Estimated 3x3 essential matrix
void calcEssentialMatrix(const Eigen::Matrix3d &K, const Eigen::Matrix3d &F, Eigen::Matrix3d &E);

// Extracting the camera pose from the calculated essential matrix.
// 4 mathmatically viable combinations of rotational matrix<->translation vector pairs are obtained.
// Estimated pairs are stored in the same order in their respective container.
//
// @param E                 3x3 Essential matrix.
//
// @param R                 Estimated rotation matrices
// @param t                 Estimated translation vectors
void extractCameraPose(const Eigen::Matrix3d &E, std::vector<Eigen::Matrix3d> &R, std::vector<Eigen::Vector3d> &t);

// Given the estimated rotation matrix<->translation vector pairs and a set of 3D points for each case, 
// find the pair that results in the most 3D poins with z >0.
// Only one rotation<->matrix pair is obtained as the physically most viable solution.
//
// @param R                 Rotational matrices
// @param t                 tranlational vectors
// @param pts3D_H           Container of 3D points in homogeneous coordinate, 
//                          each 3D point set in the container corresponding to points 
//                          triangulated with each rot<->tran pair.
//     
// @param best_R            Selected rotation matrix
// @param best_t            Selected translation vector
// @param best_pts3D_H      Selected 3D point set for selected rot<->tran pair.
void disambiguatePose(const std::vector<Eigen::Matrix3d> &R,
                      const std::vector<Eigen::Vector3d> &t, 
                      const std::vector<Eigen::MatrixXd> &pts3D_H, 
                      Eigen::Matrix3d &best_R, 
                      Eigen::Vector3d &best_t, 
                      Eigen::MatrixXd &best_pts3D_H);