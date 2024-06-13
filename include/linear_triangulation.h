#pragma once

#include <vector>
#include <Eigen/Dense>

// Linear triangulation, the underlying assumption is that the same intrinsic matrix is shared by different view points.
// i.e., Same camera used to capture pictures from different poses.
// Given intrinsic matrix, poses for two distinct views, and matched pixels between two views, this function calculates
// the corresponding 3D points.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param R1                3x3 Rotation matrix of view1.
// @param t1                3x1 translation vector of view1.
// @param R2                3x3 Rotation matrix of view2.
// @param t2                3x1 translation vector of view2.
// @param pts2D_1           Matched pixels of view1.
// @param pts2D_2           Matched pixels of view2.

// @param pts3D_H           Triangulated 3D points in homogeneous coordinate.
void linearTriangulation(const Eigen::Matrix3d &K, 
                         const Eigen::Matrix3d &R1,
                         const Eigen::Vector3d &t1,
                         const Eigen::Matrix3d &R2, 
                         const Eigen::Vector3d &t2, 
                         const Eigen::MatrixXd &pts2D_1, 
                         const Eigen::MatrixXd &pts2D_2, 
                         Eigen::MatrixXd &pts3D_H);