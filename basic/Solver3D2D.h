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
    protected:
        int max3Dnum, min3Dnum;
        int inlierNum;
        double maxFeatureDis;
    public:
        typedef shared_ptr<Solver3D2D> Ptr;
        Mat descriptorsMap;
        vector<cv::Point3f> points3D;
        KeyFrame::Ptr keyFrame2;

        Solver3D2D(
                cv::Ptr<cv::DescriptorMatcher> matcher,
                double disThresRatio = Config::get<double>("Solver3D2D.Matcher.disThresRatio"),
                double disThresMin = Config::get<double>("Solver3D2D.Matcher.disThresMin"),
                int max3Dnum = Config::get<int>("Solver3D2D.max3Dnum"),
                int min3Dnum = Config::get<int>("Solver3D2D.min3Dnum"),
                double maxFeatureDis = Config::get<double>("Solver3D2D.maxFeatureDis")
        ) :
                Matcher(matcher, disThresRatio, disThresMin),
                max3Dnum(max3Dnum), min3Dnum(min3Dnum), maxFeatureDis(maxFeatureDis) {}

        bool solve(Map::Ptr map, KeyFrame::Ptr keyFrame2);

        double getInlierRatio();

        int getInlierNum();

    private:
        Mat solvePose();

    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
