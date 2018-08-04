//
// Created by pidan1231239 on 18-7-13.
//

#include "KeyFrame.h"
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sky {

    KeyFrame::KeyFrame(const Camera::Ptr &camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D) :
            camera(camera), image(image) {
#ifdef DEBUG
        cout << "KeyFrame: detectAndCompute... ";
#endif
        feature2D->detectAndCompute(image, cv::noArray(), keyPoints, descriptors);
#ifdef DEBUG
        cout << keyPoints.size() << " keypoints found" << endl;
#endif
    }

    Vector3d KeyFrame::getCamCenterEigen() const {
        return Tcw.inverse().translation();
    }

    cv::Mat KeyFrame::getTcwMatCV(int rtype) const {
        cv::Mat TcwCV, TcwCVR;
        cv::eigen2cv(Tcw.matrix(), TcwCV);
        TcwCV.convertTo(TcwCVR, rtype);
        return TcwCVR;
    }

    cv::Mat KeyFrame::getTcw34MatCV(int rtype) const {
        auto TcwCV = getTcwMatCV(rtype);
        Mat Tcw34;
        TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
        return Tcw34;
    }

    cv::Mat KeyFrame::getTwcMatCV(int rtype) const {
        cv::Mat TwcCV, TwcCVR;
        cv::eigen2cv(Tcw.inverse().matrix(), TwcCV);
        TwcCV.convertTo(TwcCVR, rtype);
        return TwcCVR;
    }

    MapPointPtr KeyFrame::getMapPoint(int i) const {
        auto it = mapPoints.find(i);
        if (it != mapPoints.end()) {
            return it->second;
        }
/*        if (hasMapPoint(i))
            return mapPoints[i];*/
        else {
            cerr << "KeyFrame: getMapPoint failed! KeyPoint "
                 << i << " has no corresponding mapPoint" << endl;
            return nullptr;
        }
    }

    bool KeyFrame::setMapPoint(int i, MapPointPtr &mapPoint) {
        if (hasMapPoint(i)) {
            mapPoints[i] = mapPoint;
            return true;
        }
        return false;
    }

}
