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
    protected:
        Solver2D2D solver2D2D;
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

        bool step(KeyFrame::Ptr frame) {
            if (frameInterval == -1) {
#ifdef DEBUG
                cout << "Initializer: Initialization already ready!" << endl;
#endif
                return true;
            }
            if (!keyFrame1) {
                keyFrame1 = frame;
            } else {
                ++frameInterval;
                if (frameInterval > maxFrameInterval) {
                    frameInterval = 0;
                    keyFrame1 = frame;
#ifdef DEBUG
                    cout << "Initializer: Initializing keyFrame1 reseted, push another frame..." << endl;
#endif
                } else if (frameInterval < minFrameInterval) {
#ifdef DEBUG
                    cout << "Initializer: FrameInterval not enough, push another frame..." << endl;
#endif
                } else {
                    solver2D2D.solve(keyFrame1, frame);
                    if (solver2D2D.getInlierRatio() > inlierThresRatio) {
                        Triangulater triangulater;
                        initialMap = solver2D2D.triangulate();
                        if (initialMap->mapPoints.size() >= minMapPointNum) {
                            keyFrame2 = frame;
                            frameInterval = -1;
#ifdef DEBUG
                            cout << "Initializer: Initialization ready!" << endl;
#endif
                            return true;
                        } else {
#ifdef DEBUG
                            cout << "Initializer: MapPointNum not enough, push another frame..." << endl;
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
