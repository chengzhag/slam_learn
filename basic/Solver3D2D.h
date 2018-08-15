//
// Created by pidan1231239 on 18-7-15.
//

#ifndef SLAM_LEARN_SOLVER3D2D_H
#define SLAM_LEARN_SOLVER3D2D_H

#include "common_include.h"
#include "LocalMap.h"
#include <opencv2/opencv.hpp>
#include "Matcher.h"
#include "Config.h"

namespace sky {

    class Solver3D2D {

    private:
        int inlierNum;
        float inlierRatio;

        int max3Dnum, min3Dnum;
        float max3Ddis;
        int minInlierNum;
        float minInlierRatio;

        Matcher matcher;

        Mat descriptorsMap;
        vector<cv::Point3f> points3D;
        vector<MapPoint::Ptr> pointsCandi;
        LocalMap::Ptr localMap;
        KeyFrame::Ptr keyFrame2;

        Mat indexInliers;

    public:
        typedef shared_ptr<Solver3D2D> Ptr;

        Solver3D2D(
                LocalMap::Ptr localMap,
                int minInlierNum = Config::get<int>("Solver3D2D.minInlierNum"),
                float minInlierRatio = Config::get<float>("Solver3D2D.minInlierRatio"),
                int max3Dnum = Config::get<int>("Solver3D2D.max3Dnum"),
                int min3Dnum = Config::get<int>("Solver3D2D.min3Dnum"),
                float max3Ddis = Config::get<float>("Solver3D2D.max3Ddis")
        ) :
                localMap(localMap),
                minInlierNum(minInlierNum),
                minInlierRatio(minInlierRatio),
                max3Dnum(max3Dnum), min3Dnum(min3Dnum), max3Ddis(max3Ddis),
                matcher(
                        Config::get<float>("Solver3D2D.Matcher.rankRatio"),
                        Config::get<float>("Solver3D2D.Matcher.disThresRatio"),
                        Config::get<float>("Solver3D2D.Matcher.disThresMin"),
                        Config::get<float>("Solver3D2D.Matcher.testRatio")
                        ){
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << "Solver3D2D: Initializing..." << endl;
            printVariable(minInlierNum);
            printVariable(minInlierRatio);
            printVariable(max3Dnum);
            printVariable(min3Dnum);
            printVariable(max3Ddis);

            printVariable(matcher.rankRatio);
            printVariable(matcher.disThresRatio);
            printVariable(matcher.disThresMin);
            printVariable(matcher.testRatio);
        }

        bool solve(const KeyFrame::Ptr &keyFrame2);

        void addFrame2inliers(bool add2mapPoints = true);

        inline float getInlierRatio() const {
            return inlierRatio;
        }

        inline int getInlierNum() const {
            return inlierNum;
        }

    private:

        void solvePose();

    };

}


#endif //SLAM_LEARN_SOLVER3D2D_H
