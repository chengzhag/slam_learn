//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"
#include "Solver2D2D.h"

namespace sky {

    class LocalMap {
    protected:
        Solver2D2D solver2D2D;
    public:
        Map::Ptr map;
        typedef shared_ptr<LocalMap> Ptr;

        LocalMap(const cv::Ptr<cv::DescriptorMatcher> matcher) :
                solver2D2D(matcher) {}
                
        void addFrame(KeyFrame::Ptr frame);

    };


}


#endif //SLAM_LEARN_LOCALMAP_H
