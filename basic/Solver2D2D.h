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
        int inlierNum;
        double inlierRatio;

        int minInlierNum;
        double minInlierRatio;

        KeyFrame::Ptr keyFrame1, keyFrame2;

        Mat inlierMask;

    public:
        typedef shared_ptr<Solver2D2D> Ptr;

        Solver2D2D(cv::Ptr<cv::DescriptorMatcher> matcher,
                   int minInlierNum = Config::get<int>("Solver2D2D.minInlierNum"),
                   double minInlierRatio = Config::get<double>("Solver2D2D.minInlierRatio")
        ) :
                Matcher(matcher),
                minInlierNum(minInlierNum),
                minInlierRatio(minInlierRatio)
        {}

        bool solve(KeyFrame::Ptr &keyFrame1, KeyFrame::Ptr &keyFrame2, bool saveResult = true);

        double getInlierRatio();

        int getInlierNum();

        Map::Ptr triangulate();


    private:

        void solvePose(bool saveResult);


    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
