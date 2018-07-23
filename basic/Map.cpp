//
// Created by pidan1231239 on 18-7-13.
//

#include "Map.h"

#ifdef CLOUDVIEWER_DEBUG

#include <pcl/common/common_headers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#endif

namespace sky {

    void Map::addFrame(KeyFrame::Ptr frame) {
        if (frame)
            keyFrames.push_back(frame);
    }

    void Map::addMapPoint(MapPoint::Ptr mapPoint) {
        if (mapPoint)
            mapPoints.push_back(mapPoint);
    }


}