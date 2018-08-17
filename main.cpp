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
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Usage: run_vo parameter_file" << endl;
        return 1;
    }
    Config::setParameterFile(argv[1]);

    string imagesFolder(Config::get<string>("datasetDir"));
#ifdef DEBUG
    std::cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Reading images from: " + imagesFolder
              << std::endl;
#endif
    vector<string> imagesDir;

    if (!imagesFolder.empty()) {
        using namespace boost::filesystem;

        path dirPath(imagesFolder);
        if (not exists(dirPath) or not is_directory(dirPath)) {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "Cannot open directory: " << imagesFolder
                 << endl;
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
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "Unable to find valid files in images directory (\"" << imagesFolder << "\")." << endl;
            return false;
        }

        sort(imagesDir.begin(), imagesDir.end());
    }

    Camera::Ptr camera = Camera::Ptr(new Camera);

    LocalMap::Ptr localMap(new LocalMap);
    VO vo(camera,
          localMap
    );

    auto startIndex = Config::get<int>("startIndex");

    for (int i = startIndex; i < imagesDir.size(); ++i) {
        Mat image = imread(imagesDir[i]);
        //cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << image.type() << "\t" << image.depth() << endl;
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
             << "==============Adding image: " + imagesDir[i]
             << "==============" << endl;
#endif
/*#ifdef CVVISUAL_DEBUGMODE
        cvv::showImage(image, CVVISUAL_LOCATION, "Adding image: " + imageDir, "");
#endif*/
        if (!vo.step(image)) {
#ifdef DEBUG
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "SLAM: VO lost! " << endl;
#endif
            cvv::finalShow();
            break;
        }

        boost::this_thread::sleep_for(boost::chrono::milliseconds(30));
    }

    while (1) {
        cv::waitKey(1);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(30));
    }


    return 0;
}