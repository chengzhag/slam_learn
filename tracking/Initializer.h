//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_INITIALIZER_H
#define SLAM_LEARN_INITIALIZER_H

#include "common_include.h"
#include "Map.h"
#include "KeyFrame.h"
#include <opencv2/opencv.hpp>
#include "Solver2D2D.h"
#include "Config.h"

namespace sky {

    class Initializer {
    private:
        Solver2D2D solver2D2D;
        int maxFrameInterval, minFrameInterval, frameInterval = 0, minMapPointNum;
    public:
        typedef shared_ptr<Initializer> Ptr;
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Map::Ptr initialMap;

        Initializer(cv::Ptr<cv::DescriptorMatcher> matcher,
                    int maxFrameInterval = Config::get<int>("Initializer.maxFrameInterval"),
                    int minFrameInterval = Config::get<int>("Initializer.minFrameInterval"),
                    int minMapPointNum = Config::get<int>("Initializer.minMapPointNum")) :
                solver2D2D(matcher),
                maxFrameInterval(maxFrameInterval),
                minFrameInterval(minFrameInterval),
                minMapPointNum(minMapPointNum) {}

        bool step(KeyFrame::Ptr frame);

    };

}


#endif //SLAM_LEARN_INITIALIZER_H
