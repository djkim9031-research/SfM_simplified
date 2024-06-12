# Use Ubuntu 20.04 LTS as the base image
FROM ubuntu:20.04

# Avoid prompts from apt
ARG DEBIAN_FRONTEND=noninteractive

# Update and install development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    pkg-config \
    libgtk-3-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libopenexr-dev \
    libtbb-dev \
    libeigen3-dev \
    libpcl-dev \
    g++ \
    gcc \
    python3-dev \
    python3-numpy \
    # Add Google glog for logging
    libgoogle-glog-dev \
    # Add SuiteSparse and CXSparse (optional, uncomment if needed)
    libsuitesparse-dev \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Clone OpenCV and OpenCV Contrib
RUN git clone https://github.com/opencv/opencv.git /opt/opencv \
 && git clone https://github.com/opencv/opencv_contrib.git /opt/opencv_contrib

# Checkout a specific version of OpenCV and OpenCV Contrib
RUN cd /opt/opencv && git checkout 4.5.2 \
 && cd /opt/opencv_contrib && git checkout 4.5.2

# Build OpenCV
RUN mkdir /opt/opencv/build && cd /opt/opencv/build \
 && cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/opt/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON \
    -D CMAKE_CXX_STANDARD=17 .. \
 && make -j$(nproc) \
 && make install

# Cleanup unnecessary files to reduce image size
RUN rm -rf /opt/opencv /opt/opencv_contrib

# Clone and build Ceres Solver
RUN git clone https://ceres-solver.googlesource.com/ceres-solver /opt/ceres-solver \
 && mkdir /opt/ceres-solver/build && cd /opt/ceres-solver/build \
 && cmake .. \
 && make -j$(nproc) \
 && make install \
 && rm -rf /opt/ceres-solver

# Set the working directory in the container
WORKDIR /code

# Copy local files under the current dir into the container
COPY . /code

# Set the default command to bash
CMD ["/bin/bash"]
