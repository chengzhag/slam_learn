//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_SOLVER3D2D_H
#define SLAM_LEARN_SOLVER3D2D_H

#include "common_include.h"
#include "Map.h"
#include <opencv2/opencv.hpp>
#include "Matcher.h"

namespace sky {

    class Solver3D2D : protected Matcher {
    private:
        int max3Dnum, min3Dnum;
        int inlierNum;
        double maxPointDis;
    public:
        typedef shared_ptr<Solver3D2D> Ptr;
        Mat descriptorsMap;
        vector<cv::Point3f> points3D;
        KeyFrame::Ptr keyFrame2;

        Solver3D2D(cv::Ptr<cv::DescriptorMatcher> matcher,
                   double disThresRatio = 5, double disThresMin = 200,
                   int max3Dnum = 200, int min3Dnum = 20, double maxPointDis = 20) :
                Matcher(matcher, disThresRatio, disThresMin),
                max3Dnum(max3Dnum), min3Dnum(min3Dnum), maxPointDis(maxPointDis) {}

        bool solve(Map::Ptr map, KeyFrame::Ptr keyFrame2);

        double getInlierRatio();

    private:
        Mat solvePose();

    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
