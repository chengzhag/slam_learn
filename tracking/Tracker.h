//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_TRACKER_H
#define SLAM_LEARN_TRACKER_H

#include "common_include.h"
#include "LocalMap.h"
#include "Solver3D2D.h"
#include "Config.h"

namespace sky {

    class Tracker {

    private:
        float minKeyFrameDis, maxKeyFrameDis;
        int minKeyFrameInlierNum;
        LocalMap::Ptr localMap;
        Solver3D2D solver3D2D;
        KeyFrame::Ptr frame;

    public:
        typedef shared_ptr<Tracker> Ptr;

        Tracker(
                LocalMap::Ptr localMap,
                float minKeyFrameDis = Config::get<float>("Tracker.minKeyFrameDis"),
                float maxKeyFrameDis = Config::get<float>("Tracker.maxKeyFrameDis"),
                int minKeyFrameInlierNum = Config::get<int>("Tracker.minKeyFrameInlierNum")
        ) :
                localMap(localMap),
                minKeyFrameDis(minKeyFrameDis),
                maxKeyFrameDis(maxKeyFrameDis),
                minKeyFrameInlierNum(minKeyFrameInlierNum){}

        void step(const KeyFrame::Ptr &frame);

    private:
        bool isKeyFrame();


    };

}

#endif //SLAM_LEARN_TRACKER_H
