//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_TRIANGULATER_H
#define SLAM_LEARN_TRIANGULATER_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include "Config.h"

namespace sky {


    class Triangulater {
    private:
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Map::Ptr map;

    public:
        float maxDisRatio, minDisRatio, maxReprojErr;

        Triangulater(
                float maxDisRatio = Config::get<float>("Triangulater.maxDisRatio"),
                float minDisRatio = Config::get<float>("Triangulater.minDisRatio"),
                float maxReprojErr = Config::get<float>("Triangulater.maxReprojErr")
        ) :
                maxDisRatio(maxDisRatio),
                minDisRatio(minDisRatio),
                maxReprojErr(maxReprojErr) {
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Triangulater: Initializing..." << endl;
            printVariable(maxDisRatio);
            printVariable(minDisRatio);
            printVariable(maxReprojErr);
        }

        Map::Ptr
        triangulate(const KeyFrame::Ptr &keyFrame1,
                    const KeyFrame::Ptr &keyFrame2,
                    const vector<cv::DMatch> &matches,
                    Mat inlierMask = Mat());

        void viewReprojInCVV() const;

    private:

        void convAndAddMappoints(const Map::Ptr &map, const Mat &inlierMask,
                                 const Mat &points4D, const vector<cv::DMatch> &matches);

        bool isGoodPoint(const MapPoint::Ptr &mapPoint) const;

    };
}

#endif //SLAM_LEARN_TRIANGULATER_H
