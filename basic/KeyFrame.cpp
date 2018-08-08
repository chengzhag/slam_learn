//
// Created by pidan1231239 on 18-7-13.
//

#include "KeyFrame.h"
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sky {

    KeyFrame::KeyFrame(const Camera::Ptr &camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D) :
            camera(camera), image(image) {
        feature2D->detectAndCompute(image, cv::noArray(), keyPoints, descriptors);
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
        auto it = index2mapPoints.left.find(i);
        if (it != index2mapPoints.left.end()) {
            return it->second;
        } else {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "KeyFrame: getMapPoint failed! KeyPoint "
                 << i << " has no corresponding mapPoint" << endl;
            return nullptr;
        }
    }

    bool KeyFrame::setMapPoint(int i, MapPointPtr &mapPoint) {
        auto it = index2mapPoints.left.find(i);
        if (it != index2mapPoints.left.end()) {
            index2mapPoints.left.replace_data(it, mapPoint);
            return true;
        }else{
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "KeyFrame: setMapPoint failed! KeyFrame has no mapPoint "
                 << i << endl;
        }
        return false;
    }

}
