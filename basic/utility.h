//
// Created by pidan1231239 on 18-7-26.
//

#ifndef SLAM_LEARN_UTILITY_H
#define SLAM_LEARN_UTILITY_H

#include <opencv2/opencv.hpp>

namespace sky {

    template<class K, class T, class H, class P, class A,
            template<class, class, class, class, class> class M>
    inline bool mapHas(const M<K, T, H, P, A> &map, const K &key) {
        return map.find(key) != map.end();
    };

    template<class K, class H, class P, class A,
            template<class, class, class, class> class S>
    inline bool setHas(const S<K, H, P, A> &set, const K &key) {
        return set.find(key) != set.end();
    };

    //计算距离
    template<typename T>
    T disBetween(const cv::Point_<T> &p1, const cv::Point_<T> &p2) {
        auto diff = p1 - p2;
        return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    }



}

#endif //SLAM_LEARN_UTILITY_H
