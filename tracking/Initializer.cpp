//
// Created by pidan1231239 on 18-7-13.
//

#include "Initializer.h"

namespace sky{

    bool Initializer::step(KeyFrame::Ptr frame) {
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
                    BA ba;
                    ba(initialMap, {BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
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

}
