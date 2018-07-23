//
// Created by pidan1231239 on 18-7-23.
//

#ifndef SLAM_LEARN_MAPVIEWER_H
#define SLAM_LEARN_MAPVIEWER_H

#include "Map.h"
#include "common_include.h"
#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace sky {

    class MapViewer {

    public:
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

        MapViewer() :
                viewer("3D Viewer"),
                cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {

            viewer.setBackgroundColor(0.8, 0.8, 0.8);
            viewer.addPointCloud(cloud, "Triangulated Point Cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    3,
                                                    "Triangulated Point Cloud");
            viewer.addCoordinateSystem(1.0);
        }

        void run() {
#ifdef CLOUDVIEWER_DEBUG
            boost::function0<void> f = boost::bind(&MapViewer::threadFunc, this);
            boost::thread t(f);
#endif
        }

        void update(Map::Ptr map) {
            if(!map)
                return;
#ifdef CLOUDVIEWER_DEBUG
#ifdef DEBUG
            cout << "MapViewer: Visualizing Point Could..." << endl;
            cout << "\t" << map->keyFrames.size() << " keyFrames" << endl;
            cout << "\t" << map->mapPoints.size() << " mapPoints" << endl;
#endif
            cloud->clear();
            for (auto point:map->mapPoints) {
                pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
                pointXYZ.x = point->pos(0);
                pointXYZ.y = point->pos(1);
                pointXYZ.z = point->pos(2);
                cloud->push_back(pointXYZ);
            }
            cloud->width = cloud->size();
            cloud->height = 1;
            viewer.updatePointCloud(cloud, "Triangulated Point Cloud");

            auto frame=map->keyFrames.back();
            if(lastFrame!=frame){
                addFrame(frame);
                lastFrame=frame;
            }
#endif
        }

        ~MapViewer() {}

    private:

        void threadFunc() {
            while (!viewer.wasStopped()) {
                viewer.spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds(100));
            }
        }

        Frame::Ptr lastFrame;
        void addFrame(Frame::Ptr frame, string camName = "") {
#ifdef CLOUDVIEWER_DEBUG
            //添加一帧的位姿
            Eigen::Matrix4f camPose;
            auto T_c_w = frame->Tcw.inverse().matrix();
            for (int i = 0; i < camPose.rows(); ++i)
                for (int j = 0; j < camPose.cols(); ++j)
                    camPose(i, j) = T_c_w(i, j);
            viewer.addCoordinateSystem(1.0, Eigen::Affine3f(camPose), camName);
#endif
        }
    };

}


#endif //SLAM_LEARN_MAPVIEWER_H
