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
void readImageSet(const std::string imageSetPath, std::vector<std::vector<Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>>> &imageSet);
void extractMatchesFromFile(const std::string matchSetPath, const unsigned int num_total_images, Eigen::MatrixXd &feature_x, Eigen::MatrixXd &feature_y, Eigen::MatrixXd &feature_flag, Eigen::MatrixXd &feature_descriptor);
