//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_TRACKER_H
#define SLAM_LEARN_TRACKER_H

#include "common_include.h"
#include "LocalMap.h"
#include "Solver3D2D.h"

namespace sky {

    using namespace cv;

    class Tracker {
    private:
        double keyFrameMinInlierRatio;
        //int minLocalMapPointsNum;
        double minKeyFrameDis;
    protected:
        LocalMap::Ptr localMap;
        Solver3D2D solver3D2D;
    public:
        typedef shared_ptr<Tracker> Ptr;

        Tracker(LocalMap::Ptr localMap, cv::Ptr<DescriptorMatcher> matcher,
                double keyFrameMinInlierRatio = 0.1,
                int minLocalMapPointsNum = 1000,
                double minKeyFrameDis = 2) :
                localMap(localMap), solver3D2D(matcher),
                keyFrameMinInlierRatio(keyFrameMinInlierRatio),
                //minLocalMapPointsNum(minLocalMapPointsNum),
                minKeyFrameDis(minKeyFrameDis) {}

        void step(KeyFrame::Ptr frame);

        bool isKeyFrame(KeyFrame::Ptr frame);


    };

}

#endif //SLAM_LEARN_TRACKER_H
