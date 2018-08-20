//
// Created by pidan1231239 on 18-7-23.
//

#include "MapViewer.h"

namespace sky {

    void MapViewer::update(const Map::Ptr &map, const Map::Ptr &localMap) {
#ifdef CLOUDVIEWER_DEBUG
        boost::mutex::scoped_lock lockUpdate(updateMutex);
        if (!localMap)
            return;
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
             << "MapViewer: map has " << map->keyFrames.size() << " keyFrames and "
             << map->mapPoints.size() << " mapPoints; "
             << "localMap has " << localMap->keyFrames.size() << " keyFrames and "
             << localMap->mapPoints.size() << " mapPoints. " << endl;
#endif
        //更新整体地图点云
        updateCloud(map, cloudMap);
        //更新局部地图点云
        updateCloud(localMap, cloudLocal);

        viewer.removeAllCoordinateSystems();
        //更新整体地图帧
        int i = 0;
        for (auto &frame:map->keyFrames) {
            addFrame(frame, 1, "frame" + to_string(++i));
        }
        //更新局部地图帧
        for (auto &frame:localMap->keyFrames) {
            addFrame(frame, 2, "frame" + to_string(++i));
        }

        //更新相机位姿
        if (trackCurrFrame) {
            auto target = localMap->getLastFrame()->getCamCenterEigen();
            viewer.setCameraPosition(target[0], target[1] - disCamera, target[2],
                                     target[0], target[1], target[2],
                                     0, 0, 1);
        }
        lockUpdate.unlock();

        //暂停
        if (updateWait) {
            while (!wait4keyDown);
            wait4keyDown = false;
        }

#endif
    }

#ifdef CLOUDVIEWER_DEBUG

    void MapViewer::updateCloud(
            const Map::Ptr &map,
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
        string id;
        if (cloud == cloudLocal) {
            id = "cloudLocal";
        } else if (cloud == cloudMap) {
            id = "cloudMap";
        }

        cloud->clear();
        if (drawNorms)
            viewer.removeAllShapes();
        int i = 0;
        for (auto &point:map->mapPoints) {
            for (int j = 0; j < 3; ++j) {
                if (point->pos[j] == NAN)
                    continue;
            }
            pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
            pointXYZ.x = point->pos(0);
            pointXYZ.y = point->pos(1);
            pointXYZ.z = point->pos(2);
            cloud->push_back(pointXYZ);

            //更新法线
            if (drawNorms) {
                auto pointStart = point->pos;
                auto pointTo = point->pos + point->norm;
                viewer.addLine(pcl::PointXYZ(pointStart[0], pointStart[1], pointStart[2]),
                               pcl::PointXYZ(pointTo[0], pointTo[1], pointTo[2]),
                               1, 1, 1,
                               id + "Normal" + to_string(++i));
            }
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        cloud->is_dense = true;
        viewer.updatePointCloud(cloud, id);
    }

    void MapViewer::threadFunc() {
        while (!viewer.wasStopped()) {
            boost::mutex::scoped_lock lock(updateMutex);
            viewer.spinOnce();
            lock.unlock();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(15));
        }
    }

    void MapViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "Return" && event.keyDown()) {
            wait4keyDown = true;
        }
        if (event.getKeySym() == "space" && event.keyDown()) {
            wait4keyDown = true;
            updateWait = !updateWait;
        }
        if (event.getKeySym() == "Up" && event.keyDown()) {
            disCamera *= 2.0 / 3.0;
        }
        if (event.getKeySym() == "Down" && event.keyDown()) {
            disCamera *= 3.0 / 2.0;
        }
        if (event.getKeySym() == "t" && event.keyDown()) {
            trackCurrFrame = !trackCurrFrame;
        }
    }

#endif

#ifdef CLOUDVIEWER_DEBUG

    void MapViewer::addFrame(const KeyFrame::Ptr &frame, double size, string camName) {
        //添加一帧的位姿图标
        Eigen::Matrix4f camPose;
        //auto T_c_w = frame->Tcw.inverse();
        auto T_c_w = frame->Tcw.inverse().matrix();
        for (int i = 0; i < camPose.rows(); ++i)
            for (int j = 0; j < camPose.cols(); ++j)
                camPose(i, j) = T_c_w(i, j);
        viewer.addCoordinateSystem(size, Eigen::Affine3f(camPose), camName);
        /*Vector3d pointStart(0, 0, 0), pointTo(0, 0, 1);
        pointStart = T_c_w * pointStart;
        pointTo = T_c_w * pointStart;
        viewer.addLine(pcl::PointXYZ(pointStart[0], pointStart[1], pointStart[2]),
                       pcl::PointXYZ(pointTo[0], pointTo[1], pointTo[2]),
                       1, 1, 1,
                       camName);*/
    }

#endif
}
