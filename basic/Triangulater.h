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
        float maxDisRatio, minDisRatio, maxProjDis;
        KeyFrame::Ptr keyFrame1, keyFrame2;

    public:

        Triangulater(
                float maxDisRatio = Config::get<float>("Triangulater.maxDisRatio"),
                float maxProjDis = Config::get<float>("Triangulater.maxProjDis")
        ) :
                maxDisRatio(maxDisRatio),
                maxProjDis(maxProjDis) {}

        Map::Ptr
        triangulate(KeyFrame::Ptr keyFrame1, KeyFrame::Ptr keyFrame2, vector<cv::DMatch> &matches,
                    Mat inlierMask = Mat());

    private:

        void convAndAddMappoints(Map::Ptr map, const Mat &inlierMask,
                                 const Mat &points4D, const vector<cv::DMatch> &matches);

        bool isGoodPoint(MapPoint::Ptr mapPoint);

        template<typename T>
        T point2dis(cv::Point_<T> p1, cv::Point_<T> p2) {
            auto diff = p1 - p2;
            return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
        }

    };
}

#endif //SLAM_LEARN_TRIANGULATER_H
