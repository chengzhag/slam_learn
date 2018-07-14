//
// Created by pidan1231239 on 18-7-12.
//

#include "Frame.h"

namespace sky {

    Frame::~Frame() {

    }


    Vector3d Frame::getCamCenterEigen() const {
        return Tcw.inverse().translation();
    }

    cv::Mat Frame::getTcwMatCV(int rtype) {
        cv::Mat TcwCV, TcwCVR;
        cv::eigen2cv(Tcw.matrix(), TcwCV);
        TcwCV.convertTo(TcwCVR, rtype);
        return TcwCVR;
    }

    cv::Mat Frame::getTcw34MatCV(int rtype) {
        auto TcwCV = getTcwMatCV(rtype);
        Mat Tcw34;
        TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
        return Tcw34;
    }

    cv::Mat Frame::getTwcMatCV(int rtype) {
        cv::Mat TwcCV, TwcCVR;
        cv::eigen2cv(Tcw.inverse().matrix(), TwcCV);
        TwcCV.convertTo(TwcCVR, rtype);
        return TwcCVR;
    }

    bool Frame::isInFrame(const Vector3d &pt_world) {
        // cout<<"pt_world = "<<endl<<pt_world<<endl;
        Vector3d p_cam = camera->world2camera(pt_world, Tcw);
        // cout<<"P_cam = "<<p_cam.transpose()<<endl;
        if (p_cam(2, 0) < 0) return false;
        Vector2d pixel = camera->world2pixel(pt_world, Tcw);
        // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
        return pixel(0, 0) > 0 && pixel(1, 0) > 0
               && pixel(0, 0) < cols
               && pixel(1, 0) < rows;
    }

}

