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
            mapPointsPos[mapPoints] = mapPoints->getPosMatx13<double>();
        }
        //加载frameExtrinsics和cameraIntrinsics
        for (auto &frame:map->keyFrames) {
            auto angleAxis = frame->getAngleAxisWcMatxCV<double>();
            auto t = frame->Tcw.translation();
            frameExtrinsics[frame] = cv::Matx23d(angleAxis(0), angleAxis(1), angleAxis(2),
                                                 t[0], t[1], t[2]);
            if (!mapHas(cameraIntrinsics, frame->camera))
                cameraIntrinsics[frame->camera] = cv::Matx14d(
                        frame->camera->fx, frame->camera->fy, frame->camera->cx, frame->camera->cy);
        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Map loaded. "
             << mapPointsPos.size() << " map points, "
             << frameExtrinsics.size() << " keyFrames, "
             << cameraIntrinsics.size() << " cameras. " << endl;
#endif

    }

    void BA::bundleAdjustment() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Bundle Adjustment... " << endl;
#endif
        ceres::Problem problem;

        for (auto &frameExtrinsic:frameExtrinsics)
            problem.AddParameterBlock(frameExtrinsic.second.val, 6);
        auto itKeyFrame = map->keyFrames.begin();
        if (hasMode(Mode_Fix_First_Frame) || hasMode(Mode_Fix_First_2Frames))
            problem.SetParameterBlockConstant(frameExtrinsics[*itKeyFrame].val);
        if (hasMode(Mode_Fix_First_2Frames)) {
            if (++itKeyFrame == map->keyFrames.end()) {
                cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                     << "BA: Bundle Adjustment Failed! Only one frame in the map! "
                     << endl;
                return;
            }
            problem.SetParameterBlockConstant(frameExtrinsics[*itKeyFrame].val);
        }


        for (auto &cameraIntrinsic:cameraIntrinsics) {
            problem.AddParameterBlock(cameraIntrinsic.second.val, 4);
            if (hasMode(Mode_Fix_Intrinsic))
                problem.SetParameterBlockConstant(cameraIntrinsic.second.val);
        }

        ceres::LossFunction *lossFunction = new ceres::HuberLoss(4);
        for (auto &mapPointPos:mapPointsPos) {

            problem.AddParameterBlock(mapPointPos.second.val, 3);
            if (hasMode(Mode_Fix_Points))
                problem.SetParameterBlockConstant(mapPointPos.second.val);

            mapPointPos.first->forObservedFrames(
                    [&](auto &observedFrame) {
                        if (mapHas(frameExtrinsics, observedFrame.first)) {
                            ceres::CostFunction *costFunction =
                                    new ceres::AutoDiffCostFunction<ReprojectCost, 2, 4, 6, 3>(
                                            new ReprojectCost(observedFrame.second));
                            problem.AddResidualBlock(
                                    costFunction,
                                    lossFunction,
                                    cameraIntrinsics[observedFrame.first->camera].val,            // Intrinsic
                                    frameExtrinsics[observedFrame.first].val,  // View Rotation and Translation
                                    mapPointPos.second.val          // Point in 3D space
                            );
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
                      << "    #views: " << frameExtrinsics.size() << "\n"
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
            for (auto &mapPointPos:mapPointsPos) {
                mapPointPos.first->setPos(mapPointPos.second);
            }
        //写frameExtrinsics
        for (auto &frameExtrinsic:frameExtrinsics) {
            frameExtrinsic.first->setTcw(frameExtrinsic.second);
        }
        //写cameraIntrinsics
        if (!hasMode(Mode_Fix_Intrinsic))
            for (auto &cameraIntrinsic:cameraIntrinsics) {
                cameraIntrinsic.first->setIntrinsic(cameraIntrinsic.second);
            }
    }

    void BA::clear() {
        map = nullptr;

        cameraIntrinsics.clear();
        frameExtrinsics.clear();
        mapPointsPos.clear();
    }

}
