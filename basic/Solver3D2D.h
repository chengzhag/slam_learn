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
        float inlierRatio;

        int max3Dnum, min3Dnum;
        float max3Ddis;
        int minInlierNum;
        float minInlierRatio;

        Mat descriptorsMap;
        vector<cv::Point3f> points3D;
        KeyFrame::Ptr keyFrame2;

        Mat indexInliers;

    public:
        typedef shared_ptr<Solver3D2D> Ptr;

        Solver3D2D(
                int minInlierNum = Config::get<int>("Solver3D2D.minInlierNum"),
                float minInlierRatio = Config::get<float>("Solver3D2D.minInlierRatio"),
                int max3Dnum = Config::get<int>("Solver3D2D.max3Dnum"),
                int min3Dnum = Config::get<int>("Solver3D2D.min3Dnum"),
                float max3Ddis = Config::get<float>("Solver3D2D.max3Ddis")
        ) :
                minInlierNum(minInlierNum),
                minInlierRatio(minInlierRatio),
                max3Dnum(max3Dnum), min3Dnum(min3Dnum), max3Ddis(max3Ddis) {}

        bool solve(const Map::Ptr &map, const KeyFrame::Ptr &keyFrame2);

        inline float getInlierRatio() {
            return inlierRatio;
        }

        inline int getInlierNum() {
            return inlierNum;
        }

    private:

        void solvePose();

    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
