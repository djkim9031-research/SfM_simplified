#include "sfm.h"

namespace{
    // Define a hash function for std::pair<int, int> to use it as a key in std::unordered_map
    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            auto hash1 = std::hash<T1>{}(pair.first);
            auto hash2 = std::hash<T2>{}(pair.second);
            // Magic number 0x9e3779b9 is used to spread out the bits of the hash
            return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
        }
    };

} // end of namespace

void sfm_pipeline(const std::string& data_path){

    //____________________________________________________________________________________________________
    // Step 1: Read image set
    //____________________________________________________________________________________________________
    std::vector<std::vector<Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>>> images;
    readImageSet(data_path, images);
    unsigned int num_images = images.size();

    //____________________________________________________________________________________________________
    // Step 2: Extract features and corresponding feature flags. Feature descriptors are NOT 
    //         SIFT descriptors. They are simply color of the pixel to be used for point cloud
    //         (which is not yet implemented).
    //____________________________________________________________________________________________________

    Eigen::MatrixXd feature_xs, feature_ys, feature_flags, feature_descriptors;
    extractMatchesFromFile(data_path, num_images, feature_xs, feature_ys, feature_flags, feature_descriptors);


    //____________________________________________________________________________________________________
    // Step 3: Given a pair of images, run the RANSAC algorithm to sample the most inliers of 
    //         the 2D pixel matches, which simultaneously obtains the fundamental matrix for each pair.
    //         (However, only the fundamental matrix of one pair is needed). 
    //         The inlier features will be used to construct the 3D point cloud of the scene.
    //____________________________________________________________________________________________________

    // Technically, not needed. But helps to debug.
    std::unordered_map<std::pair<int, int>, Eigen::Matrix3d, pair_hash> f_matices;
    
    // Construct the matrix to collect inliers from RANSAC
    int num_total_features = feature_flags.rows();
    Eigen::MatrixXi filtered_feature_flags = Eigen::MatrixXi::Zero(num_total_features, num_images);

    // Run pair-wise RANSAC and find the best fundamental matrix and corresponding inliers for each pair.
    // O(num_total_features * num_images * num_images) process, as this is an exhaustive search.
    for(unsigned int i = 0; i < num_images - 1; ++i){
        for(unsigned int j = i + 1; j< num_images; ++j){
            
            // Collect all the matched features for the current image pair.
            std::vector<int> curr_pair_feat_idx;
            for(unsigned int k = 0; k < num_total_features; ++k){
                if(feature_flags(k, i) && feature_flags(k, j)){
                    // This is when there is a correspondence between image i and image j
                    curr_pair_feat_idx.push_back(k);
                }
            }

            // Construct matched 2D points of image i and image j.
            Eigen::MatrixXd pts1(curr_pair_feat_idx.size(), 2);
            Eigen::MatrixXd pts2(curr_pair_feat_idx.size(), 2); 
            for(unsigned int k = 0; k < curr_pair_feat_idx.size(); ++k){
                pts1(k, 0) = feature_xs(curr_pair_feat_idx[k], i);
                pts1(k, 1) = feature_ys(curr_pair_feat_idx[k], i);
                pts2(k, 0) = feature_xs(curr_pair_feat_idx[k], j);
                pts2(k, 1) = feature_ys(curr_pair_feat_idx[k], j);
            }

            // Run the 8-point algorithm to calculate the fundamental matrix.
            // This runs the RANSAC and finds the best inlier set.
            if(curr_pair_feat_idx.size() > 7){
                Eigen::Matrix3d estimated_F;
                std::vector<int> filtered_indices;
                RANSAC(pts1, pts2, curr_pair_feat_idx, estimated_F, filtered_indices);

                // TODO: use a better logger.
                std::cout<<"At image: "<<i<<", "<<j<<" || Number of inliers: "<<filtered_indices.size()<<"/"<<curr_pair_feat_idx.size()<<std::endl;

                // Technically, not needed. Only need for first pair to estimate E.
                // However, useful for debugging.
                f_matices[std::make_pair(i, j)] = estimated_F;
                
                // Populate the `filtered_feature_flags`
                for(int k = 0; k < filtered_indices.size(); ++k){
                    filtered_feature_flags(filtered_indices[k], i) = 1;
                    filtered_feature_flags(filtered_indices[k], j) = 1;
                }
            }
        }
    } 

    //____________________________________________________________________________________________________
    // Step 4: Caculate the poses of the first image pair and reconstruct the 3D point cloud.
    //         Calculation of the first pair relies on epipolar geometry, since the only
    //         available information is 2D matches of the given pair.
    //
    // TODO: a. Read the calibration matrix from .txt file
    //       b. Select the first pair based on the maximum number of inliers
    //       c. Better logging function
    //____________________________________________________________________________________________________

    Eigen::Matrix3d K, E;
    K << 568.996140852, 0, 643.21055941,
         0, 568.988362396, 477.982801038,
         0, 0, 1;
    Eigen::Matrix3d F = f_matices[std::make_pair(0, 1)];
    calcEssentialMatrix(K, F, E);

    // Set of temporary rotation matrix, translation vector combinations.
    // 4 possible solutions are obtained from the essential matrix.
    // Only 1 physical most probably solution is obtained.
    std::vector<Eigen::Matrix3d> R_temp; 
    std::vector<Eigen::Vector3d> t_temp;
    extractCameraPose(E, R_temp, t_temp);

    // Collect inlier feature matches (2D points) of the first image pair.
    std::vector<int> filtered_indices;
    for(unsigned int i = 0; i < filtered_feature_flags.rows(); ++i){
        if(filtered_feature_flags(i, 0) && filtered_feature_flags(i, 1)){
            filtered_indices.push_back(i);
        }
    }

    Eigen::MatrixXd pts2D_0(filtered_indices.size(), 2); // inlier matched 2D points of view 0 (origin)
    Eigen::MatrixXd pts2D_1(filtered_indices.size(), 2); // inlier matched 2D points of view 1
    for(unsigned int i = 0; i < filtered_indices.size(); ++i){
        pts2D_0(i, 0) = feature_xs(filtered_indices[i], 0);
        pts2D_0(i, 1) = feature_ys(filtered_indices[i], 0);
        pts2D_1(i, 0) = feature_xs(filtered_indices[i], 1);
        pts2D_1(i, 1) = feature_ys(filtered_indices[i], 1);
    }

    // Pose of the view 0 (origin)
    Eigen::Matrix3d R0 = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t0(0.0, 0.0, 0.0);

    // First, triangulate matched 2D points using the view 0's pose (absolute),
    // and mathematically estimated combinations of view 1's pose (to be disambiguated).
    std::vector<Eigen::MatrixXd> pts3D_H_under_each_pose_combination;
    for(unsigned int i = 0; i < R_temp.size(); ++ i){
        Eigen::MatrixXd pts3D_H_under_current_pose_combination;
        linearTriangulation(K, R0, t0, R_temp[i], t_temp[i], pts2D_0, pts2D_1, pts3D_H_under_current_pose_combination);
        pts3D_H_under_each_pose_combination.push_back(pts3D_H_under_current_pose_combination);
    }

    // Disambugate the view 1's pose.
    Eigen::Matrix3d R1;
    Eigen::Vector3d t1;
    Eigen::MatrixXd triangulated_pts3D_H;
    disambiguatePose(R_temp, t_temp, pts3D_H_under_each_pose_combination, R1, t1, triangulated_pts3D_H);


    // Caculate the reprojection error (linear triangulation).
    std::cout<<"[View 0 - View 1] Reprojection error: "<<
               reprojectionError(triangulated_pts3D_H, 
                                 pts2D_0,
                                 pts2D_1,
                                 R0,
                                 t0,
                                 R1,
                                 t1,
                                 K) << std::endl;
    
    // Nonlinear optimization, and calculate the reprojection error after nonlinear optimization.
    Eigen::MatrixXd optimized_pts3D_H;
    nonLinearTriangulation(K, pts2D_0, pts2D_1, triangulated_pts3D_H, R0, t0, R1, t1, optimized_pts3D_H);
    std::cout<<"[View 0 - View 1] Reprojection error after nonlinear optimization: "<<
               reprojectionError(optimized_pts3D_H, 
                                 pts2D_0,
                                 pts2D_1,
                                 R0,
                                 t0,
                                 R1,
                                 t1,
                                 K) << std::endl;

    // Create data structures with Eigen::Matrix to register other cameras
    Eigen::MatrixXd all_pts3D = Eigen::MatrixXd::Zero(num_total_features, 3);
    Eigen::MatrixXi valid_pts = Eigen::MatrixXi::Zero(num_total_features, 1);

    std::vector<Eigen::Matrix3d> reconstructed_R_set;
    std::vector<Eigen::Vector3d> reconstructed_t_set;
    int count = 0;
    for(unsigned int i = 0; i < filtered_indices.size(); ++i){
        
        all_pts3D.row(filtered_indices[i]) = optimized_pts3D_H.block(i, 0, 1, 3);

        // Set valid_pts to zero where z is negative.
        if(optimized_pts3D_H(i, 2) < 0){
            valid_pts(filtered_indices[i]) = 0;
            continue;
        }

        valid_pts(filtered_indices[i]) = 1;
        count++;
    }

    reconstructed_R_set.push_back(R0);
    reconstructed_R_set.push_back(R1);
    reconstructed_t_set.push_back(t0);
    reconstructed_t_set.push_back(t1);

    std::cout<<"[View 0 - View 1] Registration completed with "<<count<<" new 3D points added."<<std::endl;
            
    //____________________________________________________________________________________________________
    // Step 5: Register the remaining views. This involves calculating the poses of remaining views,
    //         and reconstruct & populate 3D points further with subsequent registration.
    //
    // TODO: 1. Local bundle adjustment
    //       2. During triangulation process, if the current 2D point already has 3D point from earlier
    //          reconstruction, update its position based on reprojection error-based weights, for instance.
    //       3. Update the data structure for reconstructed_R_set, _t_set - if PNP fails/skips for any view,
    //          the current scheme fails due to index mismatch.
    //       4. Finally extract 3D points from all_pts3D and valid_pts to visualize. Also add color
    //          Currently visualization of the reconstructed views are tested only on python pcl.
    //       5. Update the pose reconstruction logic - currently done between (view j, view 0)
    //____________________________________________________________________________________________________

    std::cout<<std::endl;
    std::cout<<"Registering the remaining cameras..."<<std::endl;
    for(unsigned int i = 2; i < num_images; ++i){
         std::cout<<"[View "<<i<<" - View 0] Registering a new view with the reference view."<<std::endl;

         std::vector<int> feature_with_valid_3D_pt_idxs;
         for(unsigned int j = 0; j < num_total_features; ++j){
            if(valid_pts(j, 0) && filtered_feature_flags(j, i)){
                // This logic needs to be updated, since view i with view 0 may not have common filtered_feature_flags
                // This approach will drift the pose error as the reconstruction progress further into non-sharing views.

                // Check if a valid 3D point exists for the current 2D point in view j.
                feature_with_valid_3D_pt_idxs.push_back(j);
            }
         }

         if(feature_with_valid_3D_pt_idxs.size() < 6){
            // At least 6 points are needed to run PnP pose estimation.
            std::cout<<"[View "<<i<<" - View 0] Not enough correspondences found, skipping this view"<<std::endl;
            continue;
         }

         Eigen::MatrixXd curr_view_pts2D(feature_with_valid_3D_pt_idxs.size(), 2);
         Eigen::MatrixXd curr_view_pts3D(feature_with_valid_3D_pt_idxs.size(), 3);
         for(unsigned int j = 0; j< feature_with_valid_3D_pt_idxs.size(); ++j){
            curr_view_pts2D(j, 0) = feature_xs(feature_with_valid_3D_pt_idxs[j], i);
            curr_view_pts2D(j, 1) = feature_ys(feature_with_valid_3D_pt_idxs[j], i);
            
            curr_view_pts3D.row(j) = all_pts3D.row(feature_with_valid_3D_pt_idxs[j]);
         }

         // PNP pose estimation
         Eigen::Matrix3d estimated_R;
         Eigen::Vector3d estimated_t;
         RANSAC(K, curr_view_pts2D, curr_view_pts3D, estimated_R, estimated_t);

         std::cout<<"[View "<<i<<" - View 0] Pose estimation - reprojection error: "<<
                    reprojectionErrorPnP(K, 
                                         curr_view_pts2D, 
                                         curr_view_pts3D,
                                         estimated_R,
                                         estimated_t)<< std::endl;

        // Nonlinear optimization for PnP
        nonLinearPnP(K, curr_view_pts2D, curr_view_pts3D, estimated_R, estimated_t);
        std::cout<<"[View "<<i<<" - View 0] Pose estimation - reprojection error after nonlinear optimization: "<<
                    reprojectionErrorPnP(K, 
                                         curr_view_pts2D, 
                                         curr_view_pts3D,
                                         estimated_R,
                                         estimated_t)<< std::endl;

        // Collect the optimized pose for the current view.
        reconstructed_R_set.push_back(estimated_R);
        reconstructed_t_set.push_back(estimated_t);

        // Triangulation of the current view with respect to the already registered views.
        for(unsigned int j = 0; j < i; ++j){
            std::vector<int> curr_ref_common_feature_idxs;
            for(unsigned int k = 0; k < num_total_features; ++k){
                if(filtered_feature_flags(k, j) && filtered_feature_flags(k, i)){
                    curr_ref_common_feature_idxs.push_back(k);
                }
            }

            if(curr_ref_common_feature_idxs.size() < 8){
                // At least 8 points are needed to robustly triangulate.
                continue;
            }

            // j is the index of already registered view.
            // i is the index of the current view.
            Eigen::MatrixXd pts2D_j(curr_ref_common_feature_idxs.size(), 2);
            Eigen::MatrixXd pts2D_i(curr_ref_common_feature_idxs.size(), 2);
            for(unsigned int k = 0; k < curr_ref_common_feature_idxs.size(); ++k){
                pts2D_j(k, 0) = feature_xs(curr_ref_common_feature_idxs[k], j);
                pts2D_j(k, 1) = feature_ys(curr_ref_common_feature_idxs[k], j);
                pts2D_i(k, 0) = feature_xs(curr_ref_common_feature_idxs[k], i);
                pts2D_i(k, 1) = feature_ys(curr_ref_common_feature_idxs[k], i);
            }

            Eigen::MatrixXd estimated_pts3D_H;
            linearTriangulation(K, 
                                reconstructed_R_set[j], 
                                reconstructed_t_set[j], 
                                estimated_R, 
                                estimated_t,
                                pts2D_j,
                                pts2D_i,
                                estimated_pts3D_H);

            // Caculate the reprojection error (linear triangulation).
            std::cout<<"[View "<<i<<" - View "<<j<<"] Reprojection error: "<<
                        reprojectionError(estimated_pts3D_H, 
                                          pts2D_j,
                                          pts2D_i,
                                          reconstructed_R_set[j],
                                          reconstructed_t_set[j],
                                          estimated_R,
                                          estimated_t,
                                          K) << std::endl;
            
            // Nonlinear optimization, and calculate the reprojection error after nonlinear optimization.
            nonLinearTriangulation(K, 
                                   pts2D_j, 
                                   pts2D_i, 
                                   estimated_pts3D_H,
                                   reconstructed_R_set[j],
                                   reconstructed_t_set[j], 
                                   estimated_R, 
                                   estimated_t, 
                                   optimized_pts3D_H);

            std::cout<<"[View "<<i<<" - View "<<j<<"] Reprojection error after nonlinear optimization: "<<
                        reprojectionError(optimized_pts3D_H, 
                                          pts2D_j,
                                          pts2D_i,
                                          reconstructed_R_set[j],
                                          reconstructed_t_set[j],
                                          estimated_R,
                                          estimated_t,
                                          K) << std::endl;

            count = 0;
            for(unsigned int k = 0; k < curr_ref_common_feature_idxs.size(); ++k){
                all_pts3D.row(curr_ref_common_feature_idxs[k]) = optimized_pts3D_H.block(k, 0, 1, 3);

                if(optimized_pts3D_H(k, 2) < 0){
                    valid_pts(curr_ref_common_feature_idxs[k]) = 0;
                    count++;
                    continue;
                }

                valid_pts(curr_ref_common_feature_idxs[k]) = 1;
            }

            std::cout<<"[View "<<i<<" - View "<<j<<"] Updated "<<curr_ref_common_feature_idxs.size() - count<< " points\n"<<std::endl;
        }
        std::cout<<"____________________________________________________________________________________________________"<<std::endl;
    }

    count = 0;
    for(unsigned int i = 0; i < num_total_features; ++i){
        if(valid_pts(i, 0)){
            count++;
        }
    }

    std::cout<<"Registration of all view completed, total "<<count<<" unique 3D points reconstructed"<<std::endl;;
    return;
}