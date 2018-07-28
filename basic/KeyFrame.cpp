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

    bool KeyFrame::proj2frame(const Vector3d &pt_world, Vector2d &pixelColRow) const {
        Vector3d p_cam = camera->world2camera(pt_world, Tcw);
        pixelColRow = camera->world2pixel(pt_world, Tcw);
        // cout<<"pt_world = "<<endl<<pt_world<<endl;
        // cout<<"P_pixel = "<<pixelColRow.transpose()<<endl<<endl;
        if (p_cam(2, 0) < 0) return false;
        return pixelColRow(0, 0) > 0
               && pixelColRow(1, 0) > 0
               && pixelColRow(0, 0) < image.cols
               && pixelColRow(1, 0) < image.rows;
    }

    float KeyFrame::getDis2(const Sophus::Vector3d &coor) const {
        Sophus::Vector3d coorFrom = Tcw.translation();
        float dis = 0;
        for (int i = 0; i < 3; ++i)
            dis += pow(coorFrom[i] - coor[i], 2);
        return sqrt(dis);
    }

    MapPoint::Ptr KeyFrame::getMapPoint(int i) const {
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

    bool KeyFrame::setMapPoint(int i, MapPoint::Ptr &mapPoint) {
        if (hasMapPoint(i)) {
            mapPoints[i] = mapPoint;
            return true;
        }
        return false;
    }

}
