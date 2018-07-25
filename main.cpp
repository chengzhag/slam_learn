#include <iostream>
#include "common_include.h"

#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>

#include "SLAM.h"

using namespace cv;
using namespace sky;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }
    Config::setParameterFile(argv[1]);

    string imagesFolder(Config::get<string>("datasetDir"));
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
            Config::get<float>("Camera.fx"),
            Config::get<float>("Camera.fy"),
            Config::get<float>("Camera.cx"),
            Config::get<float>("Camera.cy")
    ));

    LocalMap::Ptr localMap(new LocalMap);
    VO vo(camera,
          localMap
            //ORB::create(Config::get<int>("ORB.nfeatures"))
            //xfeatures2d::SIFT::create(Config::get<int>("ORB.nfeatures"), 3, 0.04, 10)
    );
    MapViewer mapViewer;
    mapViewer.run();

    for (auto &imageDir:imagesDir) {
        Mat image = imread(imageDir);
#ifdef DEBUG
        cout << endl << "==============Adding image: " + imageDir << "==============" << endl;
#endif
/*#ifdef CVVISUAL_DEBUGMODE
        cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + imageDir, "");
#endif*/
        vo.step(image);
        mapViewer.update(localMap->map);
        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }


    return 0;
}