#include <iostream>
#include "common_include.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "SLAM.h"

using namespace cv;
using namespace sky;

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

    Camera::Ptr camera = Camera::Ptr(new Camera(
            718.856, 718.856, 607.1928, 185.2157
    ));
    auto matcher = DescriptorMatcher::create("BruteForce");
    LocalMap::Ptr localMap(new LocalMap(matcher));
    VO vo(camera,
          matcher,
          ORB::create(500),
          localMap
    );

    for (auto &imageDir:imagesDir) {
        Mat image = imread(imageDir);
#ifdef DEBUG
        cout << endl << "==============Adding image: " + imageDir << "==============" << endl;
#endif
#ifdef CVVISUAL_DEBUGMODE
        cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + imageDir, "");
#endif
        vo.step(image);
        if (vo.getState() == 1) {
            //localMap->map->visInCloudViewer();
            //break;
        }
    }


    return 0;
}