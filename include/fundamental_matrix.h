#pragma once

#include <vector>
#include <Eigen/Dense>

// Estimate fundamental matrix given a pair of 2D image points.
// @param pts1              Nx2 image points from view 1 (N should be >= 8)
// @param pts2              Nx2 image points from view 2 (N should be >= 8) 
//
// @param F                 Estimated 3x3 fundamental matrix
void estimateFundamentalMatrix(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2, Eigen::Matrix3d &F);
