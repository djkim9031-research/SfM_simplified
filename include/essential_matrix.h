#pragma once
#include <Eigen/Dense>

// Calculating the essential matrix from camera calibration matrix and a fundamental matrix.
// This assumes the camera is already calibrated and the intrinsic matrix is known.
//
// @param K         3x3 Camera intrinsic matrix.
// @param F         3x3 Fundamental matrix
// 
// @return E        Estimated 3x3 essential matrix
void calcEssentialMatrix(const Eigen::Matrix3d &K, const Eigen::Matrix3d &F, Eigen::Matrix3d &E);