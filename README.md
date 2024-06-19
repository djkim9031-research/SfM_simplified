# Simplified implementation of Structure from Motion (SfM)

Given a set of 6 images with fisheye lens distortion corrected, and SIFT keypoints features + matchings for each image pair, this implementation utilize the following steps to reconstruct 3D scene. The camera calibration matrix is provided.

1. Epipolar geometry (Essential matrix) to decompose the pose of 2nd image with respect to the 1st (at origin).
2. Triangulation to generate 3D points using 1st and 2nd image pair.
3. Nonlinear optimization using Ceres solver to minimize the reprojection error of the estimated 3D points.
4. For each next image and already registered image, given the observable 3D points in the pair and corresponding 2D matched points, PnP algorithm is used to obtain the image's pose with respect to the 1st (at origin).
5. Given the pose of the image, triangulation is performed to estimate 3D points.
6. Nonlinear optimization using Ceres solver to minimize the reprojection error.
6. Step 4 and 6 are repeated until all image pairs are covered.


## Prerequisites

Ensure you have the following software installed on your system:

- Docker
- CMake
- A C++ compiler (This repo has been tested with `g++` version 9.4.0)

## Getting Started

### Building the Docker Container

1. **Clone the repository:**

   ```
   $git clone https://github.com/djkim9031-research/SfM_simplified.git
   $cd SfM_simplified
   ```
   
2. **Build the docker container:**
   
   ```
   $docker build -t sfm:latest .
   ```
   
3. **Run the Docker container:**

   ```
   $docker run -v /path/to/SfM_simplified:/code -it --name sfm_volume sfm:latest
   ```

### Building the CMake and Run 


1. **Navigate to `code` and run the build commands**:
   
   ```
   $cd code
   $mkdir build && cd build
   $cmake ..
   $make -j${nproc}
   ```
2. **Run the SfM pipeline:**

   All the data should be under `data` directory.
   ```
   $./SfM ../data
   ```

## Project structure

Here is the overall project structure:
```
/SfM_simplified
├── CMakeLists.txt                    # CMake configuration file.
├── Dockerfile                        # Docker configuration file.
├── README.md                         # Project README file.
├── src                               # Source directory.
│   ├── main.cpp                      # Main entry point of the application.
│   ├── algorithms_PnP.cpp            # Implementation of PnP algorithms, Ceres pose optimization.
│   ├── essential_matrix.cpp          # Calculation of essential matrix, pose extraction.
│   ├── fundamental_matrix.cpp        # Calculation of fundamental matrix.
│   ├── image_reader.cpp              # Implementation of generic data reader pipeline.
│   ├── linear_triangulation.cpp      # Calculation of linear triangulation.
│   ├── nonlinear_triangulation.cpp   # Calculation of nonlinear triangulation, Ceres optimization.
│   ├── projection_utils.cpp          # Implementation/Helper function for projection matrices.
│   ├── ransac.cpp                    # Implementation of RANSAC overload functions for relevant use cases.
│   └── sfm.cpp                       # General sfm 3D scene reconstruction pipeline.
│── include
│   ├── algorithms_PnP.h              # Header file for algorithms_PnP.cpp file.
│   ├── essential_matrix.h            # Header file for essential_matrix.cpp file.
│   ├── fundamental_matrix.h          # Header file for fundamental_matrix.cpp file.
│   ├── image_reader.h                # Header file for image_reader.cpp file.
│   ├── linear_triangulation.h        # Header file for linear_triangulation.cpp file.
│   ├── nonlinear_triangulation.h     # Header file for nonlinear_triangulation.cpp file.
│   ├── projection_utils.h            # Header file for projection_utils.cpp file.
│   ├── ransac.h                      # Header file for ransac.cpp file.
│   └── sfm.h                         # Header file for sfm.cpp file.
└── data                              # Directory containing input data for the SfM pipeline
```
