//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_SLAM_H
#define SLAM_LEARN_SLAM_H

#include "common_include.h"
#include "Map.h"
#include "KeyFrame.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include <unordered_map>
#include "BA.h"
#include "VO.h"
#include "LocalMap.h"
#include "MapViewer.h"
#include "Config.h"

namespace sky {

    using namespace cv;

    class SLAM {
    private:
        MapViewer::Ptr mapViewer;

        Map::Ptr map;
        Camera::Ptr camera;
        LocalMap::Ptr localMap;
        VO vo;

    public:
        SLAM() :
                mapViewer(new MapViewer),
                map(new Map),
                camera(new Camera),
                localMap(new LocalMap(map,mapViewer)),
                vo(camera, localMap) {}

        inline bool step(const Mat &image) {
            return vo.step(image);
            mapViewer->update(localMap->localMap);
        }

    };

}

#endif //SLAM_LEARN_SLAM_H
