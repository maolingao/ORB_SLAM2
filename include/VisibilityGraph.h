#pragma once
#include"Map.h"
#include"Tracking.h"

namespace ORB_SLAM2 {
class Tracking;
class Map;
// class KeyPoint;
// class KeyFrame;

class Intrinsic{
    cv::Mat* mK;
    cv::Mat* mDistCoef;
};

class VisibilityGraph{
    VisibilityGraph(Map& map);
    // KFpose = keyframe timestamp + T_wc
    using KFpose = std::pair<const double*, const const cv::Mat*>;
    // all keyframes
    std::vector<KFpose> mKfPoses;

    // KeyPointCoord = (x,y)
    using KeyPointCoord = std::vector<const float*>;
    // observation = keyframe timestamp + feature coordinate
    using Obs = std::pair<const double*, const KeyPointCoord>;
    // obsList = vector of observations of a lamdmark
    using ObsList = std::vector<Obs>;
    // Landmark = worldPosition + obsList
    using Landmark = std::pair<const cv::Mat*, const ObsList>;
    // all landmarks
    std::vector<Landmark> mLandmarks;
};

class Log{
    Intrinsic mIntrinsic;
    VisibilityGraph mVisibilityGraph;

};
}
