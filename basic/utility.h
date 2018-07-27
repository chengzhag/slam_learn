//
// Created by pidan1231239 on 18-7-26.
//

#ifndef SLAM_LEARN_UTILITY_H
#define SLAM_LEARN_UTILITY_H

#include <opencv2/opencv.hpp>

namespace sky {

    template<typename T>
    T point2dis(cv::Point_<T> p1, cv::Point_<T> p2) {
        auto diff = p1 - p2;
        return cv::sqrt(diff.x * diff.x + diff.y * diff.y);
    }

    template<class K, class T, class H, class P, class A,
            template<class, class, class, class, class> class M>
    inline bool mapHas(M<K, T, H, P, A> map, K key) {
        return map.find(key) != map.end();
    };

    template<class K, class H, class P, class A,
            template<class, class, class, class> class S>
    inline bool setHas(S<K, H, P, A> set, K key) {
        return set.find(key) != set.end();
    };
}

#endif //SLAM_LEARN_UTILITY_H
