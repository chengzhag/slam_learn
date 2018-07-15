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
    protected:
        LocalMap::Ptr localMap;
        Solver3D2D solver3D2D;
    public:
        typedef shared_ptr<Tracker> Ptr;

        Tracker(LocalMap::Ptr localMap,cv::Ptr<DescriptorMatcher> matcher) :
                localMap(localMap), solver3D2D(matcher) {}

        void step(KeyFrame::Ptr frame){
            solver3D2D.solve(localMap->map,frame);
            //TODO:三角化
            //判断是否插入关键帧
            if(true){
                localMap->addFrame(frame);
            }
        }


    };

}

#endif //SLAM_LEARN_TRACKER_H
