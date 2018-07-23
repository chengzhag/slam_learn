//
// Created by pidan1231239 on 18-7-12.
//

#include "Camera.h"


namespace sky {

    Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w) {
        return T_c_w * p_w;
    }

    Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w) {
        return T_c_w.inverse() * p_c;
    }

    Vector2d Camera::camera2pixel(const Vector3d &p_c) {
        return Vector2d(
                fx * p_c(0, 0) / p_c(2, 0) + cx,
                fy * p_c(1, 0) / p_c(2, 0) + cy
        );
    }

    Vector3d Camera::pixel2camera(const Vector2d &p_p, double depth) {
        return Vector3d(
                (p_p(0, 0) - cx) * depth / fx,
                (p_p(1, 0) - cy) * depth / fy,
                depth
        );
    }

    Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w) {
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Vector3d Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth) {
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }

    cv::Point2f Camera::pixel2normal(const cv::Point2d &p) const{
        return cv::Point2f
                (
                        (p.x - cx) / fx,
                        (p.y - cy) / fy
                );
    }

    float Camera::getFocalLength() {
        return (fx + fy) / 2;
    }

    cv::Point2d Camera::getPrincipalPoint() {
        return cv::Point2d(cx, cy);
    }

    cv::Matx<float, 3, 3> Camera::getKMatxCV() {
        return cv::Matx<float, 3, 3>(fx, 0, cx, 0, fy, cy, 0, 0, 1);
    }

    cv::Mat Camera::getKMatCV() {
        Mat K = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        return K;
    }

}
