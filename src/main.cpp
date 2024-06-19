#include "sfm.h"

#include <iostream>
#include <string>


void print_help(const std::string& program_name) {
    std::cerr << "Usage: " << program_name << " <data_path>\n"
              << "    <data_path>: Path to the data directory\n";
}

int main(int argc, char** argv) {

    if (argc != 2) {
        print_help(argv[0]);
        return 1;
    }

    const std::string data_path = argv[1];

    try {
        sfm_pipeline(data_path);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}