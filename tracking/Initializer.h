//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_INITIALIZER_H
#define SLAM_LEARN_INITIALIZER_H

#include "common_include.h"
#include "Map.h"
#include "Frame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include "BA.h"

#include "KeyFrame.h"
#include "Solver2D2D.h"

namespace sky {

    using namespace cv;

    class Initializer {
    public:
        typedef shared_ptr<Initializer> Ptr;
        KeyFrame::Ptr keyFrame1, keyFrame2;
        Map initialMap;
        cv::Ptr<DescriptorMatcher> matcher;

        Initializer(cv::Ptr<DescriptorMatcher> matcher,
                    double inlierThresRatio = 0.35, int maxFrameInterval = 5, int minFrameInterval = 2) :
                matcher(matcher),
                inlierThresRatio(inlierThresRatio),
                maxFrameInterval(maxFrameInterval),
                minFrameInterval(minFrameInterval) {}

        bool step(KeyFrame::Ptr keyFrame) {
            if (!keyFrame1) {
                keyFrame1 = keyFrame;
            } else {
                ++frameInterval;
                if (frameInterval > maxFrameInterval) {
                    frameInterval = 0;
                    keyFrame1 = keyFrame;
#ifdef DEBUG
                    cout << "initializing keyFrame1 reseted, push another frame..." << endl;
#endif
                } else if (frameInterval < minFrameInterval) {
#ifdef DEBUG
                    cout << "frameInterval not enough, push another frame..." << endl;
#endif
                } else {
                    Solver2D2D solver2D2D(keyFrame1, keyFrame, matcher);
                    if (solver2D2D.getInlierRatio() > inlierThresRatio) {
                        keyFrame2 = keyFrame;
                        //TODO:三角化
#ifdef DEBUG
                        cout << "initialization ready!" << endl;
#endif
                        return true;
                    }
                }
            }
            return false;
        }


    protected:
        double inlierThresRatio;
        int maxFrameInterval, minFrameInterval, frameInterval = 0;
    };

}


#endif //SLAM_LEARN_INITIALIZER_H
