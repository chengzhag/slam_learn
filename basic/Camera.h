//
// Created by pidan1231239 on 18-7-12.
//

#ifndef SLAM_LEARN_CAMERA_H
#define SLAM_LEARN_CAMERA_H

#include "common_include.h"
#include "opencv2/opencv.hpp"

namespace sky {

    class Camera {
    public:
        typedef shared_ptr<Camera> Ptr;
        float fx, fy, cx, cy;

        Camera(float fx, float fy, float cx, float cy) :
                fx(fx), fy(fy), cx(cx), cy(cy) {}

        template<typename T>
        void setIntrinsic(cv::Matx<T, 1, 4> intrinsic) {
            fx=intrinsic(0);
            fy=intrinsic(1);
            cx=intrinsic(2);
            cy=intrinsic(3);
        }

        //坐标转换

        Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);

        Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);

        Vector2d camera2pixel(const Vector3d &p_c);

        Vector3d pixel2camera(const Vector2d &p_p, double depth = 1);

        Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth = 1);

        Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);

        cv::Point2f pixel2normal(const cv::Point2d &p) const;

        //获取参数
        float getFocalLength();

        cv::Point2d getPrincipalPoint();

        cv::Matx<float, 3, 3> getKMatxCV();

        cv::Mat getKMatCV();
    };

}


#endif //SLAM_LEARN_CAMERA_H
