#include "image_reader.h"

void readImageSet(const std::string imageSetPath, std::vector<std::vector<Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>>> &imageSet){

    std::vector<fs::path> jpgFiles;
    for(const auto& entry : fs::directory_iterator(imageSetPath)){
        if(entry.path().extension() == ".jpg"){
            jpgFiles.push_back(entry.path());
        }
    }

    for(const auto& imagePath : jpgFiles){
        cv::Mat image = cv::imread(imagePath.string(), cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cout << "Could not open or find the image: " << imagePath << std::endl;
            continue;
        }

        assert(image.type() == CV_8UC3);
        Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> R(image.rows, image.cols);
        Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> G(image.rows, image.cols);
        Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> B(image.rows, image.cols);
        
        for(unsigned int i=0; i<image.rows;++i){
            for(unsigned int j=0;j<image.cols;++j){
                cv::Vec3b pixel = image.at<cv::Vec3b>(i, j);
                B(i, j) = static_cast<unsigned int>(pixel[0]); // Channel 0 (Blue)
                G(i, j) = static_cast<unsigned int>(pixel[1]); // Channel 1 (Green)
                R(i, j) = static_cast<unsigned int>(pixel[2]); // Channel 2 (Red)
            }
        }

        std::vector<Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic>> imageMatrix;
        imageMatrix.push_back(B);
        imageMatrix.push_back(G);
        imageMatrix.push_back(R);

        imageSet.push_back(imageMatrix);
        //Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> B_uint = B.cast<unsigned int>();
        //Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> G_uint = G.cast<unsigned int>();
        //Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> R_uint = R.cast<unsigned int>();
    }
};

void extractMatchesFromFile(const std::string matchSetPath, const unsigned int num_total_images, Eigen::MatrixXd &feature_x, Eigen::MatrixXd &feature_y, Eigen::MatrixXd &feature_flag, Eigen::MatrixXd &feature_descriptor){

    std::vector<Eigen::RowVectorXd> holder_x, holder_y, holder_flag;
    std::vector<Eigen::RowVector3d> holder_descriptor;

    for(unsigned int n=1; n<num_total_images; ++n){
        std::string matching_file_name = matchSetPath + "/matching" + std::to_string(n) + ".txt";
        std::ifstream file(matching_file_name);
        int nFeatures = 0; 

        if(!file.is_open()){
            std::cerr<<"Failed to open file: "<<matching_file_name<<std::endl;
            continue;
        }
        std::string line;
        while(std::getline(file, line)){
            
            if(line.find("nFeatures")!=std::string::npos){
                std::istringstream iss(line);
                std::string key;
                int value;
                iss >> key >> value;
                nFeatures = value;
                std::cout<<"file: "<<matching_file_name<<" contains "<<nFeatures<<" features."<<std::endl;
                continue;
            }

            // all ux matches with corresponding image id as col
            Eigen::RowVectorXd ux_row = Eigen::RowVectorXd::Zero(num_total_images);
            // all uy matches with corresponding image id as col
            Eigen::RowVectorXd uy_row = Eigen::RowVectorXd::Zero(num_total_images);
            // flag is 1 if there is a match
            Eigen::RowVectorXi flag_row = Eigen::RowVectorXi::Zero(num_total_images);

            std::istringstream iss(line);
            std::vector<double> features((std::istream_iterator<double>(iss)), std::istream_iterator<double>());

            int n_matches = static_cast<int>(features[0]);
            Eigen::RowVector3d color(features[1], features[2], features[3]); //r, g, b
            holder_descriptor.push_back(color);

            ux_row(n-1) = features[4];
            uy_row(n-1) = features[5];
            flag_row(n-1) = 1;

            int m = 1;
            while(n_matches>1){
                int image_id = static_cast<int>(features[m+5]);
                ux_row(image_id - 1) = features[m+6];
                uy_row(image_id - 1) = features[m+7];
                flag_row(image_id - 1) = 1;
                m += 3;
                n_matches -= 1;
            }

            holder_x.push_back(ux_row);
            holder_y.push_back(uy_row);
            holder_flag.push_back(flag_row.cast<double>());
        }
    }

    assert(holder_x.size() == holder_y.size());
    assert(holder_x.size() == holder_flag.size());
    assert(holder_x.size() == holder_descriptor.size());
    unsigned int num_features = holder_x.size();

    Eigen::MatrixXd temp_x(num_features, num_total_images);
    Eigen::MatrixXd temp_y(num_features, num_total_images);
    Eigen::MatrixXd temp_flag(num_features, num_total_images);
    Eigen::MatrixXd temp_descriptor(num_features, 3);
    for(unsigned int i=0; i<num_features; ++i){
        temp_x.row(i) = holder_x[i];
        temp_y.row(i) = holder_y[i];
        temp_flag.row(i) = holder_flag[i];
        temp_descriptor.row(i) = holder_descriptor[i];
    }

    feature_x = temp_x;
    feature_y = temp_y;
    feature_flag = temp_flag;
    feature_descriptor = temp_descriptor;
};