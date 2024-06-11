# Simplified implementation of Structure from Motion (SfM)

Given a set of 6 images with fisheye lens distortion corrected, and SIFT keypoints features + matchings for each image pair, this implementation utilize the following steps to reconstruct 3D scene. The camera calibration matrix is provided.

1. Epipolar geometry (Essential matrix) to decompose the pose of 2nd image with respect to the 1st (at origin).
2. Triangulation to generate 3D points using 1st and 2nd image pair.
3. Nonlinear optimization using Ceres solver to minimize the reprojection error of the estimated 3D points.
4. For each next image and already registered image, given the observable 3D points in the pair and corresponding 2D matched points, PnP algorithm is used to obtain the image's pose with respect to the 1st (at origin).
5. Given the pose of the image, triangulation is performed to estimate 3D points.
6. Nonlinear optimization using Ceres solver to minimize the reprojection error.
6. Step 4 and 6 are repeated until all image pairs are covered.

