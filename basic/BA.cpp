//
// Created by pidan1231239 on 18-7-13.
//

#include "BA.h"

namespace sky {

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
            frame2extrinsics[frame] = frame->getAngleAxisAndTWcMatxCV<double>();
        }

        //加载map->keyFrames之外的关键帧
        if (!hasMode(Mode_Fix_Points))
            for (auto &mapPoint:map->mapPoints) {
                mapPoint->forEachFrame(
                        [&](const KeyFrame::Ptr &frame) {
                            if (!mapHas(frame2extrinsics, frame)) {
                                otherFrame2extrinsics[frame] = frame->getAngleAxisAndTWcMatxCV<double>();
                            }
                        }
                );
            }

#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Map loaded. "
             << mapPoint2poses.size() << " map points, "
             << frame2extrinsics.size() << " keyFrames, "
             << otherFrame2extrinsics.size() << " fixedOtherKeyFrames, "
             << endl;
#endif

    }

    void BA::bundleAdjustment() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "BA: Bundle Adjustment... " << endl;
#endif
        ceres::Problem problem;

        for (auto &frame2extrinsic:frame2extrinsics)
            problem.AddParameterBlock(frame2extrinsic.second.val, 6);

        for (auto &otherFrame2extrinsic:otherFrame2extrinsics)
            problem.AddParameterBlock(otherFrame2extrinsic.second.val, 6);

        for (auto &mapPoint2pos:mapPoint2poses)
            problem.AddParameterBlock(mapPoint2pos.second.val, 3);

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

        ceres::LossFunction *lossFunction = new ceres::HuberLoss(lossFunctionScaling);
        for (auto &mapPoint2pos:mapPoint2poses) {

            if (hasMode(Mode_Fix_Points))
                problem.SetParameterBlockConstant(mapPoint2pos.second.val);

            mapPoint2pos.first->forEachFrame2index(
                    [&](const auto &frame2index) {
                        cv::Point2d pixelCoor(frame2index.first->getKeyPointCoor(frame2index.second));
                        ceres::CostFunction *costFunction =
                                new ceres::AutoDiffCostFunction<ReprojectCost, 2, 6, 3>(
                                        new ReprojectCost(pixelCoor, frame2index.first->camera));
                        if (mapHas(frame2extrinsics, frame2index.first)) {
                            problem.AddResidualBlock(
                                    costFunction,
                                    lossFunction,
                                    frame2extrinsics[frame2index.first].val,  // View Rotation and Translation
                                    mapPoint2pos.second.val          // Point in 3D space
                            );
                        } else if (this->hasMode(Mode_Fix_Points)) {
                            return;
                        } else if (mapHas(otherFrame2extrinsics, frame2index.first)) {
                            problem.SetParameterBlockConstant(otherFrame2extrinsics[frame2index.first].val);
                            problem.AddResidualBlock(
                                    costFunction,
                                    lossFunction,
                                    otherFrame2extrinsics[frame2index.first].val,  // View Rotation and Translation
                                    mapPoint2pos.second.val          // Point in 3D space
                            );
                        } else {
                            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                                 << "BA: not exist in frame2extrinsics and otherFrame2extrinsics! "
                                 << endl;
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
        int i = 0;
        for (auto itKeyFrame = map->keyFrames.begin(); itKeyFrame != map->keyFrames.end(); ++itKeyFrame, ++i) {
            if (i == 0 && (hasMode(Mode_Fix_First_Frame) || hasMode(Mode_Fix_First_2Frames)))
                continue;
            if (i == 1 && hasMode(Mode_Fix_First_2Frames))
                continue;
            auto frame = frame2extrinsics.find(*itKeyFrame);
            if (frame != frame2extrinsics.end())
                (*itKeyFrame)->setTcw(frame->second);
            else
                cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                     << "BA: Frame " << (*itKeyFrame).get() << " in map is not in frame2extrinsic! "
                     << endl;
        }
    }

    void BA::clear() {
        map = nullptr;

        frame2extrinsics.clear();
        mapPoint2poses.clear();
        otherFrame2extrinsics.clear();
    }

}
