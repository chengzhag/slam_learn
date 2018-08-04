//
// Created by pidan1231239 on 18-7-13.
//

#include "Initializer.h"
#include <opencv2/cvv.hpp>
#include "BA.h"

namespace sky {

    bool Initializer::step(const KeyFrame::Ptr &frame) {
        if (frameInterval == -1) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Initializer: Initialization already ready!"
                 << endl;
#endif
            return true;
        }
        if (!keyFrame1) {
            keyFrame1 = frame;
        } else {
            ++frameInterval;
            if (frameInterval > maxFrameInterval) {
#ifdef DEBUG
                cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                     << "Initializer: Initialization not ready. FrameInterval " << frameInterval
                     << " is larger than maxFrameInterval " << maxFrameInterval << ". Reseting keyFrame1... " << endl;
#endif
                frameInterval = 0;
                keyFrame1 = frame;
                return false;
            } else if (frameInterval < minFrameInterval) {
#ifdef DEBUG
                cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                     << "Initializer: Initialization not ready. FrameInterval " << frameInterval
                     << " is less than minFrameInterval " << minFrameInterval << endl;
#endif
                return false;
            }

            if (!solver2D2D.solve(keyFrame1, frame)) {
                cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                     << "Initializer: Initialization not ready. Solver2D2D failed!" << endl;
                return false;
            }

            initialMap = solver2D2D.triangulate();

            if (initialMap->mapPoints.size() < minMapPointNum) {
#ifdef DEBUG
                cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                     << "Initializer: Initialization not ready. mapPointNum " << initialMap->mapPoints.size()
                     << " is less than minMapPointNum " << minMapPointNum << endl;
#endif
                return false;
            }

            solver2D2D.viewInliersInCVV();
            solver2D2D.viewReprojInCVV();


            BA ba({BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
            ba.loadMap(initialMap);
            ba.ba();
            ba.writeMap();

            keyFrame2 = frame;
            frameInterval = -1;
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Initializer: Initialization ready!" << endl;
#endif
            return true;

        }
        return false;
    }

}
