//
// Created by pidan1231239 on 18-7-13.
//

#include "BA.h"

namespace sky {

    BA::BA(ModeSet modeSet) :
            modeSet(modeSet) {
        ceres_config_options.minimizer_progress_to_stdout = false;
        ceres_config_options.logging_type = ceres::SILENT;
        ceres_config_options.num_threads = 4;
        ceres_config_options.preconditioner_type = ceres::JACOBI;
        ceres_config_options.linear_solver_type = ceres::SPARSE_SCHUR;
        ceres_config_options.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
    }

    void BA::loadMap(Map::Ptr map) {
        clear();
        this->map = map;
        if (!map) {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "BA: Failed! map is empty! " << endl;
        }
        if (map->keyFrames.size() == 0) {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "BA: Failed! map has no keyFrame! " << endl;
        }
        if (map->mapPoints.size() == 0) {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "BA: Failed! map has no mapPoint! " << endl;
        }

        //加载mapPointsPos
        for (auto &mapPoints:map->mapPoints) {
            mapPoint2poses[mapPoints] = mapPoints->getPosMatx13<double>();
        }
        //加载frameExtrinsics和cameraIntrinsics
        for (auto &frame:map->keyFrames) {
            auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
            auto t = frame->Tcw.translation();
            frame2extrinsics[frame] = cv::Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                                  t[0], t[1], t[2]);
            if (!mapHas(camera2intrinsics, frame->camera))
                camera2intrinsics[frame->camera] = cv::Matx14d(
                        frame->camera->fx, frame->camera->fy, frame->camera->cx, frame->camera->cy);
        }
        //加载map->keyFrames之外的关键帧
        for (auto &mapPoint:map->mapPoints) {
            mapPoint->forEachFrame(
                    [&](const KeyFrame::Ptr &frame) {
                        if (!mapHas(frame2extrinsics, frame)) {
                            auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
                            auto t = frame->Tcw.translation();
                            otherFrame2extrinsics[frame] = cv::Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                                                       t[0], t[1], t[2]);
                            if (!mapHas(camera2intrinsics, frame->camera))
                                camera2intrinsics[frame->camera] = cv::Matx14d(
                                        frame->camera->fx, frame->camera->fy, frame->camera->cx, frame->camera->cy);
                        }
                    }
            );
        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Map loaded. "
             << mapPoint2poses.size() << " map points, "
             << frame2extrinsics.size() << " keyFrames, "
             << camera2intrinsics.size() << " cameras. " << endl;
#endif

    }

    void BA::bundleAdjustment() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Bundle Adjustment... " << endl;
#endif
        ceres::Problem problem;

        for (auto &frame2extrinsic:frame2extrinsics)
            problem.AddParameterBlock(frame2extrinsic.second.val, 6);
        auto itKeyFrame = map->keyFrames.begin();
        if (hasMode(Mode_Fix_First_Frame) || hasMode(Mode_Fix_First_2Frames))
            problem.SetParameterBlockConstant(frame2extrinsics[*itKeyFrame].val);
        if (hasMode(Mode_Fix_First_2Frames)) {
            if (++itKeyFrame == map->keyFrames.end()) {
                cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                     << "BA: Bundle Adjustment Failed! Only one frame in the map! "
                     << endl;
                return;
            }
            problem.SetParameterBlockConstant(frame2extrinsics[*itKeyFrame].val);
        }


        for (auto &camera2intrinsic:camera2intrinsics) {
            problem.AddParameterBlock(camera2intrinsic.second.val, 4);
            if (hasMode(Mode_Fix_Intrinsic))
                problem.SetParameterBlockConstant(camera2intrinsic.second.val);
        }

        ceres::LossFunction *lossFunction = new ceres::HuberLoss(4);
        for (auto &mapPoint2pos:mapPoint2poses) {

            problem.AddParameterBlock(mapPoint2pos.second.val, 3);
            if (hasMode(Mode_Fix_Points))
                problem.SetParameterBlockConstant(mapPoint2pos.second.val);

            mapPoint2pos.first->forEachFrame2index(
                    [&](const auto &observedFrame) {
                        cv::Point2d pixelCoor(observedFrame.first->getKeyPointCoor(observedFrame.second));
                        ceres::CostFunction *costFunction =
                                new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(
                                        new ReprojectCost(pixelCoor));
                        if (mapHas(frame2extrinsics, observedFrame.first)) {
                            problem.AddResidualBlock(
                                    costFunction,
                                    lossFunction,
                                    camera2intrinsics[observedFrame.first->camera].val,            // Intrinsic
                                    frame2extrinsics[observedFrame.first].val,  // View Rotation and Translation
                                    mapPoint2pos.second.val          // Point in 3D space
                            );
                        } else {
                            problem.AddResidualBlock(
                                    costFunction,
                                    lossFunction,
                                    camera2intrinsics[observedFrame.first->camera].val,            // Intrinsic
                                    otherFrame2extrinsics[observedFrame.first].val,  // View Rotation and Translation
                                    mapPoint2pos.second.val          // Point in 3D space
                            );
                            problem.SetParameterBlockConstant(otherFrame2extrinsics[observedFrame.first].val);
                        }
                    }
            );

        }


        ceres::Solver::Summary summary;
        ceres::Solve(ceres_config_options, &problem, &summary);

        if (!summary.IsSolutionUsable()) {
            std::cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Bundle Adjustment failed."
                      << std::endl;
        } else {
            // Display statistics about the minimization
            std::cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "BA: Bundle Adjustment statistics (approximated RMSE):\n"
                      << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "    #views: " << frame2extrinsics.size() + otherFrame2extrinsics.size() << "\n"
                      << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "    #residuals: " << summary.num_residuals << "\n"
                      << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "    Initial RMSE: " << std::sqrt(summary.initial_cost / summary.num_residuals) << "\n"
                      << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "    Final RMSE: " << std::sqrt(summary.final_cost / summary.num_residuals) << "\n"
                      << "[" << boost::this_thread::get_id() << "]DEBUG: "
                      << "    Time (s): " << summary.total_time_in_seconds << "\n";
        }
    }

    void BA::writeMap() {
        //写mapPointsPos
        if (!hasMode(Mode_Fix_Points))
            for (auto &mapPoint2pos:mapPoint2poses) {
                mapPoint2pos.first->setPos(mapPoint2pos.second);
            }
        //写frameExtrinsics
        for (auto &frame2extrinsic:frame2extrinsics) {
            frame2extrinsic.first->setTcw(frame2extrinsic.second);
        }
        //写cameraIntrinsics
        if (!hasMode(Mode_Fix_Intrinsic))
            for (auto &camera2intrinsic:camera2intrinsics) {
                camera2intrinsic.first->setIntrinsic(camera2intrinsic.second);
            }
    }

    void BA::clear() {
        map = nullptr;

        camera2intrinsics.clear();
        frame2extrinsics.clear();
        mapPoint2poses.clear();
    }

}
