#pragma once

#include <iostream>
#include <vector>
#include <filesystem>
#include <cassert>
#include <fstream>
#include <sstream>
#include <string>
#include <iterator>
#include <random>
#include <set>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// A helper function to read the image set from a given image set path.
//
// @param imageSetPath          Path to the image set
//
// @param imageSet              Read images
void readImageSet(const std::string imageSetPath, 
                  std::vector<std::vector<Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>>> &imageSet);

// A helper function to extract SIFT features and color descriptors from a .txt file.
// Currently, the data contains SIFT feature pixel matches and their color information.
// It is not the SIFT feature descriptor.
//
// @param matchSetPath          Path to the match sets
// @param num_total_images      Number of the total images in the dataset.
//
// @param feature_x             A set of entire pixel matches (x)
// @param feature_y             A set of entire pixel matches (y)
// @param feature_flag          Flags to indicate that there is a match between image pairs
// @param feature_descriptor    A set of entire feature descriptors (color values)
void extractMatchesFromFile(const std::string matchSetPath, 
                            const unsigned int num_total_images, 
                            Eigen::MatrixXd &feature_x, 
                            Eigen::MatrixXd &feature_y, 
                            Eigen::MatrixXd &feature_flag, 
                            Eigen::MatrixXd &feature_descriptor);
