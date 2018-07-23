//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_INITIALIZER_H
#define SLAM_LEARN_INITIALIZER_H

#include "common_include.h"
#include "Map.h"
#include "KeyFrame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "BA.h"
#include "Solver2D2D.h"

namespace sky {

    using namespace cv;

    class Initializer {
    private:
        Solver2D2D solver2D2D;
        double inlierThresRatio;
        int maxFrameInterval, minFrameInterval, frameInterval = 0, minMapPointNum;
    public:
        typedef shared_ptr<Initializer> Ptr;
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Map::Ptr initialMap;

        Initializer(cv::Ptr<DescriptorMatcher> matcher,
                    double inlierThresRatio = 0.35, int maxFrameInterval = 5, int minFrameInterval = 2,
                    int minMapPointNum = 50) :
                solver2D2D(matcher),
                inlierThresRatio(inlierThresRatio),
                maxFrameInterval(maxFrameInterval),
                minFrameInterval(minFrameInterval),
                minMapPointNum(minMapPointNum) {}

        bool step(KeyFrame::Ptr frame);

    };

}


#endif //SLAM_LEARN_INITIALIZER_H
