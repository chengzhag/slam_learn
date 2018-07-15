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
        Map::Ptr initialMap;
        cv::Ptr<DescriptorMatcher> matcher;

        Initializer(cv::Ptr<DescriptorMatcher> matcher,
                    double inlierThresRatio = 0.35, int maxFrameInterval = 5, int minFrameInterval = 2,
                    int minMapPointNum = 50) :
                matcher(matcher),
                inlierThresRatio(inlierThresRatio),
                maxFrameInterval(maxFrameInterval),
                minFrameInterval(minFrameInterval),
                minMapPointNum(minMapPointNum) {}

        bool step(KeyFrame::Ptr keyFrame) {
            if (frameInterval == -1) {
#ifdef DEBUG
                cout << "Initialization already ready!" << endl;
#endif
                return true;
            }
            if (!keyFrame1) {
                keyFrame1 = keyFrame;
            } else {
                ++frameInterval;
                if (frameInterval > maxFrameInterval) {
                    frameInterval = 0;
                    keyFrame1 = keyFrame;
#ifdef DEBUG
                    cout << "Initializing keyFrame1 reseted, push another frame..." << endl;
#endif
                } else if (frameInterval < minFrameInterval) {
#ifdef DEBUG
                    cout << "FrameInterval not enough, push another frame..." << endl;
#endif
                } else {
                    Solver2D2D solver2D2D(keyFrame1, keyFrame, matcher);
                    if (solver2D2D.getInlierRatio() > inlierThresRatio) {
                        keyFrame2 = keyFrame;
                        initialMap = solver2D2D.triangulate();
                        if (initialMap->mapPoints.size() >= minMapPointNum) {
                            frameInterval = -1;
#ifdef DEBUG
                            cout << "Initialization ready!" << endl;
#endif
                            return true;
                        } else {
#ifdef DEBUG
                            cout << "MapPointNum not enough, push another frame..." << endl;
#endif
                            return false;
                        }

                    }
                }
            }
            return false;
        }


    protected:
        double inlierThresRatio;
        int maxFrameInterval, minFrameInterval, frameInterval = 0, minMapPointNum;
    };

}


#endif //SLAM_LEARN_INITIALIZER_H
