//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_TRIANGULATER_H
#define SLAM_LEARN_TRIANGULATER_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>

namespace sky {


    class Triangulater {
    private:
        double maxDisRatio;
    public:
        KeyFrame::Ptr keyFrame1, keyFrame2;

        Triangulater(double maxDisRatio = 20) : maxDisRatio(maxDisRatio) {}

        Map::Ptr
        triangulate(KeyFrame::Ptr keyFrame1, KeyFrame::Ptr keyFrame2, vector<cv::DMatch> &matches, Mat &inlierMask);

    private:

        void convAndAddMappoints(Map::Ptr map, const Mat &inlierMask,
                                 const Mat &points4D, const vector<cv::DMatch> &matches);
    };
}

#endif //SLAM_LEARN_TRIANGULATER_H
