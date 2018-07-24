//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_SOLVER3D2D_H
#define SLAM_LEARN_SOLVER3D2D_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include "Matcher.h"
#include "Config.h"

namespace sky {

    class Solver3D2D : protected Matcher {

    private:
        int inlierNum;
        double inlierRatio;

        int max3Dnum, min3Dnum;
        double max3Ddis;
        int minInlierNum;
        double minInlierRatio;

        Mat descriptorsMap;
        vector<cv::Point3f> points3D;
        KeyFrame::Ptr keyFrame2;

        Mat indexInliers;

    public:
        typedef shared_ptr<Solver3D2D> Ptr;

        Solver3D2D(
                cv::Ptr<cv::DescriptorMatcher> matcher,
                int minInlierNum = Config::get<int>("Solver3D2D.minInlierNum"),
                double minInlierRatio = Config::get<double>("Solver3D2D.minInlierRatio"),
                int max3Dnum = Config::get<int>("Solver3D2D.max3Dnum"),
                int min3Dnum = Config::get<int>("Solver3D2D.min3Dnum"),
                double max3Ddis = Config::get<double>("Solver3D2D.max3Ddis")
        ) :
                Matcher(matcher),
                minInlierNum(minInlierNum),
                minInlierRatio(minInlierRatio),
                max3Dnum(max3Dnum), min3Dnum(min3Dnum), max3Ddis(max3Ddis) {}

        bool solve(Map::Ptr map, KeyFrame::Ptr keyFrame2);

        double getInlierRatio();

        int getInlierNum();

    private:

        void solvePose();

    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
