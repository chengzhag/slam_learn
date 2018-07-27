//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_BA_H
#define SLAM_LEARN_BA_H


#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include "Map.h"
#include "KeyFrame.h"
#include "common_include.h"
#include <unordered_map>
#include <unordered_set>
#include "utility.h"

namespace sky {

    class BA {

    public:
        typedef enum mode {
            Mode_Fix_First_Frame,
            Mode_Fix_First_2Frames,
            Mode_Fix_Points,
            Mode_Fix_Intrinsic
        } Mode;

        typedef unordered_set<Mode, std::hash<int>> ModeSet;

    private:
        ModeSet modeSet;

        Map::Ptr map;
        ceres::Solver::Options ceres_config_options;

        unordered_map<Camera::Ptr, cv::Matx14d> cameraIntrinsics;
        unordered_map<KeyFrame::Ptr, cv::Matx23d> frameExtrinsics;
        unordered_map<MapPoint::Ptr, cv::Matx13d> mapPointsPos;

    public:
        BA();

        void operator()(Map::Ptr map,
                        ModeSet modeSet = {Mode_Fix_First_Frame, Mode_Fix_Intrinsic});

    private:

        class ReprojectCost {
            cv::Point2d observation;
        public:
            ReprojectCost(const cv::Point2d &observation) : observation(observation) {}

            template<typename T>
            bool
            operator()(const T *const intrinsic, const T *const extrinsic, const T *const pos3d, T *residuals) const {
                const T *r = extrinsic;
                const T *t = &extrinsic[3];

                T pos_proj[3];
                ceres::AngleAxisRotatePoint(r, pos3d, pos_proj);

                // Apply the camera translation
                pos_proj[0] += t[0];
                pos_proj[1] += t[1];
                pos_proj[2] += t[2];

                const T x = pos_proj[0] / pos_proj[2];
                const T y = pos_proj[1] / pos_proj[2];

                const T fx = intrinsic[0];
                const T fy = intrinsic[1];
                const T cx = intrinsic[2];
                const T cy = intrinsic[3];

                // Apply intrinsic
                const T u = fx * x + cx;
                const T v = fy * y + cy;

                residuals[0] = u - T(observation.x);
                residuals[1] = v - T(observation.y);

                return true;
            }
        };

        inline bool hasMode(Mode mode) {
            return setHas(modeSet, mode);
        }

        void loadMap();

        void bundleAdjustment();

        void writeMap();

        void clear();


    };

}


#endif //SLAM_LEARN_BA_H
