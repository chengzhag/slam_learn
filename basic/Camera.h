//
// Created by pidan1231239 on 18-7-12.
//

#ifndef SLAM_LEARN_CAMERA_H
#define SLAM_LEARN_CAMERA_H

#include "common_include.h"
#include "opencv2/opencv.hpp"
#include "Config.h"

namespace sky {

    class Camera {
    public:
        typedef shared_ptr<Camera> Ptr;
        float fx, fy, cx, cy;

        Camera(float fx = Config::get<float>("Camera.fx"),
               float fy = Config::get<float>("Camera.fy"),
               float cx = Config::get<float>("Camera.cx"),
               float cy = Config::get<float>("Camera.cy")
        ) :
                fx(fx), fy(fy), cx(cx), cy(cy) {
#ifdef DEBUG
            cout << "Camera: Initializing..." << endl;
            coutVariable(fx);
            coutVariable(fy);
            coutVariable(cx);
            coutVariable(cy);
#endif
        }

        template<typename T>
        void setIntrinsic(const cv::Matx<T, 1, 4> &intrinsic) {
            fx = intrinsic(0);
            fy = intrinsic(1);
            cx = intrinsic(2);
            cy = intrinsic(3);
        }

        //坐标转换

        Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w) const;

        Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w) const;

        Vector2d camera2pixel(const Vector3d &p_c) const;

        Vector3d pixel2camera(const Vector2d &p_p, double depth = 1) const;

        Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth = 1) const;

        Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w) const;

        cv::Point2f pixel2normal(const cv::Point2d &p) const;

        //获取参数
        float getFocalLength() const;

        cv::Point2d getPrincipalPoint() const;

        cv::Matx<float, 3, 3> getKMatxCV() const;

        cv::Mat getKMatCV() const;
    };

}


#endif //SLAM_LEARN_CAMERA_H
