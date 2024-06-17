#pragma once

#include <vector>
#include <Eigen/Dense>

// Helper function to calculate the projection matrix of a given camera.
//
// @param R                 3x3 Rotation matrix of the camera.
// @param t                 3x1 translation vector of the camera.
// @param K                 3x3 Camera intrinsic matrix.
//
// @param P                 3x4 Cameera projection matrix.
void projectionMatrix(const Eigen::Matrix3d &R, 
                      const Eigen::Vector3d &t, 
                      const Eigen::Matrix3d &K, 
                      Eigen::Matrix<double, 3, 4> &P);

// Funtion to calculate the reprojection error given the estimated 3D points and pairs of 2D pixels.
//
// @param pts3D_H           Triangulated 3D points in homogeneous coordinate.
// @param pts2D_1           Matched pixels of view1.
// @param pts2D_2           Matched pixels of view2.
// @param R1                3x3 Rotation matrix of view1.
// @param t1                3x1 translation vector of view1.
// @param R2                3x3 Rotation matrix of view2.
// @param t2                3x1 translation vector of view2.
// @param K                 3x3 Camera intrinsic matrix.
//
// @return                  mean reprojection error of the current 3D points <-> 2D pair correspondences
double reprojectionError(const Eigen::MatrixXd &pts3D_H, 
                         const Eigen::MatrixXd &pts2D_1, 
                         const Eigen::MatrixXd &pts2D_2, 
                         const Eigen::Matrix3d &R1, 
                         const Eigen::Vector3d &t1, 
                         const Eigen::Matrix3d &R2, 
                         const Eigen::Vector3d &t2, 
                         const Eigen::Matrix3d &K);