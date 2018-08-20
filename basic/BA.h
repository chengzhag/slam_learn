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
            Mode_Fix_Points
        } Mode;

        typedef unordered_set<Mode, std::hash<int>> ModeSet;

    private:
        ModeSet modeSet;

        Map::Ptr map;
        ceres::Solver::Options ceres_config_options;
        double lossFunctionScaling;
        cv::Matx14d intrinsic;
        unordered_map<KeyFrame::Ptr, cv::Matx23d> frame2extrinsics;
        unordered_map<KeyFrame::Ptr, cv::Matx23d> otherFrame2extrinsics;
        unordered_map<MapPoint::Ptr, cv::Matx13d> mapPoint2poses;

    public:
        BA(ModeSet modeSet = {Mode_Fix_First_Frame},
           double lossFunctionScaling = Config::get<double>("BA.lossFunction.scaling")
        ) :
                modeSet(modeSet),
                lossFunctionScaling(lossFunctionScaling) {
            //cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Initializing..." << endl;
            //printVariable(lossFunctionScaling);
            ceres_config_options.minimizer_progress_to_stdout = false;
            ceres_config_options.logging_type = ceres::SILENT;
            ceres_config_options.num_threads = 4;
            ceres_config_options.preconditioner_type = ceres::JACOBI;
            ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
            ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
        };


        void loadMap(Map::Ptr map);

        void ba() {
            bundleAdjustment();
        }

        void writeMap();

    private:

        class ReprojectCost {
            const cv::Point2d observation;
            const Camera::Ptr camera;
        public:
            ReprojectCost(const cv::Point2d &observation, Camera::Ptr camera) :
                    observation(observation),
                    camera(camera) {}

            template<typename T>
            bool
            operator()(const T *const extrinsic, const T *const pos3d, T *residuals) const {
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

                // Apply intrinsic
                const T u = (double) camera->fx * x + (double) camera->cx;
                const T v = (double) camera->fy * y + (double) camera->cy;

                residuals[0] = u - T(observation.x);
                residuals[1] = v - T(observation.y);

                return true;
            }
        };

        inline bool hasMode(Mode mode) const {
            return setHas(modeSet, mode);
        }

        void bundleAdjustment();

        void clear();


    };

}


#endif //SLAM_LEARN_BA_H
