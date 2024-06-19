#pragma once

#include "image_reader.h"
#include "ransac.h"
#include "fundamental_matrix.h"
#include "essential_matrix.h"
#include "linear_triangulation.h"
#include "projection_utils.h"
#include "nonlinear_triangulation.h"
#include "algorithms_PnP.h"

#include <unordered_map>
#include <vector>


// Structure from Motion (SfM) construction pipeline.
// Given the images and SIFT feature matching data (currently only supports .txt)
// this function builds the SfM pipeline and reconstruct the pose of each
// viewpoint and 3D points of the scene.
//
// @param data_path         Path to all the datasets
void sfm_pipeline(const std::string& data_path);