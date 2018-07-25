//
// Created by pidan1231239 on 18-7-13.
//

#include "KeyFrame.h"
#include <opencv2/cvv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sky {

    KeyFrame::KeyFrame(const Camera::Ptr camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D) :
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

    cv::Mat KeyFrame::getTcwMatCV(int rtype) {
        cv::Mat TcwCV, TcwCVR;
        cv::eigen2cv(Tcw.matrix(), TcwCV);
        TcwCV.convertTo(TcwCVR, rtype);
        return TcwCVR;
    }

    cv::Mat KeyFrame::getTcw34MatCV(int rtype) {
        auto TcwCV = getTcwMatCV(rtype);
        Mat Tcw34;
        TcwCV(cv::Range(0, 3), cv::Range(0, 4)).convertTo(Tcw34, rtype);
        return Tcw34;
    }

    cv::Mat KeyFrame::getTwcMatCV(int rtype) {
        cv::Mat TwcCV, TwcCVR;
        cv::eigen2cv(Tcw.inverse().matrix(), TwcCV);
        TwcCV.convertTo(TwcCVR, rtype);
        return TwcCVR;
    }

    bool KeyFrame::isInFrame(const Vector3d &pt_world) {
        // cout<<"pt_world = "<<endl<<pt_world<<endl;
        Vector3d p_cam = camera->world2camera(pt_world, Tcw);
        // cout<<"P_cam = "<<p_cam.transpose()<<endl;
        if (p_cam(2, 0) < 0) return false;
        Vector2d pixel = camera->world2pixel(pt_world, Tcw);
        // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
        return pixel(0, 0) > 0 && pixel(1, 0) > 0
               && pixel(0, 0) < image.cols
               && pixel(1, 0) < image.rows;
    }

    bool KeyFrame::isInFrame(MapPoint::Ptr mapPoint){
        return isInFrame(mapPoint->pos);
    }

    float KeyFrame::getDis2(Sophus::Vector3d &coor) {
        Sophus::Vector3d coorFrom = Tcw.translation();
        float dis = 0;
        for (int i = 0; i < 3; ++i)
            dis += pow(coorFrom[i] - coor[i], 2);
        return sqrt(dis);
    }

    float KeyFrame::getDis2(KeyFrame::Ptr keyFrame) {
        return getDis2(keyFrame->Tcw.translation());
    }

    float KeyFrame::getDis2(MapPoint::Ptr mapPoint) {
        return getDis2(mapPoint->pos);
    }

}
