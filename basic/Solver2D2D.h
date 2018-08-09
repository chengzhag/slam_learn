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

    class Solver2D2D {

    private:
        int inlierNum;
        float inlierRatio;

        int minInlierNum;
        float minInlierRatio;

        Matcher matcher;

        KeyFrame::Ptr keyFrame1, keyFrame2;
        Triangulater triangulater;

        Mat inlierMask;

    public:
        typedef shared_ptr<Solver2D2D> Ptr;

        Solver2D2D(int minInlierNum = Config::get<int>("Solver2D2D.minInlierNum"),
                   float minInlierRatio = Config::get<float>("Solver2D2D.minInlierRatio")
        ) :
                minInlierNum(minInlierNum),
                minInlierRatio(minInlierRatio),
                matcher(
                        Config::get<float>("Solver2D2D.Matcher.rankRatio"),
                        Config::get<float>("Solver2D2D.Matcher.disThresRatio"),
                        Config::get<float>("Solver2D2D.Matcher.disThresMin"),
                        Config::get<float>("Solver2D2D.Matcher.testRatio")
                ) {
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Solver2D2D: Initializing..." << endl;
            printVariable(minInlierNum);
            printVariable(minInlierRatio);

            printVariable(matcher.rankRatio);
            printVariable(matcher.disThresRatio);
            printVariable(matcher.disThresMin);
            printVariable(matcher.testRatio);
        }

        bool solve(const KeyFrame::Ptr &keyFrame1, const KeyFrame::Ptr &keyFrame2, bool saveResult = true);

        void viewInliersInCVV() const;

        inline void viewReprojInCVV() const {
            triangulater.viewReprojInCVV();
        }

        inline float getInlierRatio() const {
            return inlierRatio;
        }

        inline int getInlierNum() const {
            return inlierNum;
        }

        Map::Ptr triangulate();

    private:
        void solvePose(bool saveResult);


    };

}


#endif //SLAM_LEARN_SOLVER2D2D_H
