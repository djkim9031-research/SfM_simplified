#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cassert>
#include <ceres/ceres.h>

// Linear PnP algorithm for a pose estimation, given a set of 2D<->3D point correspondences and 
// a camera instrinsic matrix.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param pts2D             2D points captured from the given camera.
// @param pts3D             3D points captured from the given camera.
//
// @param R                 Estimated 3x3 rotation matrix of the camera.
// @param t                 Estimated 3x1 translation vector of the camera.
void linearPnP(const Eigen::Matrix3d &K, 
               const Eigen::MatrixXd &pts2D, 
               const Eigen::MatrixXd &pts3D, 
               Eigen::Matrix3d &R, 
               Eigen::Vector3d &t);

// Caculate a point-wise PNP error, based on 3D point reprojection.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param ps2D              One of 2D point captured from the given camera.
// @param pt3D              Corresponding 3D point captured from the given camera.
// @param R                 3x3 rotation matrix of the camera.
// @param t                 3x1 translation vector of the camera.
//
// @return                  Reprojection error of the given 2D<->3D correspondence.
double PnPError(const Eigen::Matrix3d &K,
                const Eigen::MatrixXd &pt2D, 
                const Eigen::MatrixXd &pt3D, 
                const Eigen::Matrix3d &R, 
                const Eigen::Vector3d &B);

// Nonlinear PnP pose estimation, given a set of 2D<->3D point correspondences and 
// a camera instrinsic matrix.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param pts2D             2D points captured from the given camera.
// @param pts3D             3D points captured from the given camera.
// 
// @param R                 Estimated 3x3 rotation matrix of the camera.
// @param t                 Estimated 3x1 translation vector of the camera.
void nonLinearPnP(const Eigen::Matrix3d &K, 
                  const Eigen::MatrixXd &pts2D, 
                  const Eigen::MatrixXd &pts3D, 
                  Eigen::Matrix3d &R, 
                  Eigen::Vector3d &t);


// Caculate a point-set PNP error, based on 3D point reprojection.
//
// @param K                 3x3 Camera intrinsic matrix.
// @param ps2D              2D points captured from the given camera.
// @param pt3D              Corresponding 3D points captured from the given camera.
// @param R                 3x3 rotation matrix of the camera.
// @param t                 3x1 translation vector of the camera.
//
// @return                  Reprojection error of the given 2D<->3D correspondences.
double reprojectionErrorPnP(const Eigen::Matrix3d &K, 
                            const Eigen::MatrixXd &pts2D, 
                            const Eigen::MatrixXd &pts3D,
                            const Eigen::Matrix3d &R, 
                            const Eigen::Vector3d &t);