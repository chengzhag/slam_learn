//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_SOLVER2D2D_H
#define SLAM_LEARN_SOLVER2D2D_H

#include "common_include.h"
#include "Map.h"
#include "KeyFrame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "KeyFrame.h"
#include <algorithm>
#include "Matcher.h"
#include "Triangulater.h"

namespace sky {

    using namespace cv;

    class Solver2D2D : protected Matcher {

    private:
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Triangulater triangulater;
    public:
        typedef shared_ptr<Solver2D2D> Ptr;
        Mat inlierMask;

        Solver2D2D(cv::Ptr<DescriptorMatcher> matcher,
                   double disThresRatio = 6, double disThresMin = 300) :
                Matcher(matcher, disThresRatio, disThresMin) {}

        void solve(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2, bool saveResult = true);

        double getInlierRatio();

        Map::Ptr triangulate();


    private:

        void solvePose(bool saveResult);


    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
