//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_SOLVER2D2D_H
#define SLAM_LEARN_SOLVER2D2D_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include "Matcher.h"
#include "Triangulater.h"
#include "Config.h"

namespace sky {

    class Solver2D2D : protected Matcher {

    private:
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Triangulater triangulater;
    public:
        typedef shared_ptr<Solver2D2D> Ptr;
        Mat inlierMask;

        Solver2D2D(cv::Ptr<cv::DescriptorMatcher> matcher,
                   double disThresRatio = Config::get<double>("Solver2D2D.Matcher.disThresRatio"),
                   double disThresMin = Config::get<double>("Solver2D2D.Matcher.disThresMin")) :
                Matcher(matcher, disThresRatio, disThresMin) {}

        void solve(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2, bool saveResult = true);

        double getInlierRatio();

        Map::Ptr triangulate();


    private:

        void solvePose(bool saveResult);


    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
