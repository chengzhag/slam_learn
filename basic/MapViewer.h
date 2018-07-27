//
// Created by pidan1231239 on 18-7-23.
//

#ifndef SLAM_LEARN_MAPVIEWER_H
#define SLAM_LEARN_MAPVIEWER_H

#include "Map.h"
#include "common_include.h"

#ifdef CLOUDVIEWER_DEBUG

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Config.h"

#endif


namespace sky {

    class MapViewer {


    private:
#ifdef CLOUDVIEWER_DEBUG
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        boost::thread thread;
        boost::mutex updateMutex, wait4keyMutex;
        bool updateWait;
#endif

    public:
#ifdef CLOUDVIEWER_DEBUG

        MapViewer(bool updateWait = Config::get<int>("MapViewer.updateWait")) :
                updateWait(updateWait),
                viewer("3D Viewer"),
                cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {

            viewer.setBackgroundColor(
                    Config::get<double>("MapViewer.backgroundColor.r"),
                    Config::get<double>("MapViewer.backgroundColor.g"),
                    Config::get<double>("MapViewer.backgroundColor.b")
            );
            viewer.addPointCloud(cloud, "Triangulated Point Cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    3,
                                                    "Triangulated Point Cloud");
            viewer.initCameraParameters();
            viewer.addCoordinateSystem(1.0);

            thread = boost::thread(boost::bind(&MapViewer::threadFunc, this));
            if (updateWait) {
                viewer.registerKeyboardCallback(boost::bind(&MapViewer::keyboardEventOccurred, this, _1));
            }
        }

#endif

        void update(Map::Ptr &map);

        ~MapViewer() {}

    private:
#ifdef CLOUDVIEWER_DEBUG
        void threadFunc();

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

        void addFrame(KeyFrame::Ptr &frame, string camName = "");
#endif
    };

}


#endif //SLAM_LEARN_MAPVIEWER_H
