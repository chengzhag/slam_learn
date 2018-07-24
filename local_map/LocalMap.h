//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"
#include "Matcher.h"
#include "Triangulater.h"

namespace sky {

    class LocalMap {
    protected:
        Matcher matcher;
        Triangulater triangulater;
    public:
        Map::Ptr map;
        typedef shared_ptr<LocalMap> Ptr;

        LocalMap(const cv::Ptr<cv::DescriptorMatcher> matcher) :
                matcher(matcher) {}
                
        void addFrame(KeyFrame::Ptr frame);

    };


}


#endif //SLAM_LEARN_LOCALMAP_H
