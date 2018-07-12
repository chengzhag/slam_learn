#include <iostream>
#include "common_include.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include "opencv2/opencv.hpp"

using namespace cv;

int main() {
    string imagesFolder("dataset/data_odometry_gray/dataset/sequences/00/image_0");
#ifdef DEBUG
    std::cout << "Reading images from: " + imagesFolder << std::endl;
#endif
    vector<string> imagesDir;

    if (!imagesFolder.empty()) {
        using namespace boost::filesystem;

        path dirPath(imagesFolder);
        if (not exists(dirPath) or not is_directory(dirPath)) {
            cerr << "Cannot open directory: " << imagesFolder << endl;
            return false;
        }

        for (directory_entry &x : directory_iterator(dirPath)) {
            string extension = x.path().extension().string();
            boost::algorithm::to_lower(extension);
            if (extension == ".jpg" or extension == ".png") {
                imagesDir.push_back(x.path().string());
            }
        }

        if (imagesDir.size() <= 0) {
            cerr << "Unable to find valid files in images directory (\"" << imagesFolder << "\")." << endl;
            return false;
        }

        sort(imagesDir.begin(), imagesDir.end());
    }

    for (auto &imageDir:imagesDir) {
        Mat image = imread(imageDir);
        imshow("frames", image);
        waitKey(1);
    }

    return 0;
}