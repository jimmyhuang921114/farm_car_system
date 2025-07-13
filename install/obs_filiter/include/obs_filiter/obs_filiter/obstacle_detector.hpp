#pragma once
#include <opencv2/core.hpp>
#include "plane_estimator.hpp"  // 正确包含

namespace obs_filiter {

class ObstacleDetector {
public:
    struct Parameters {
        float ground_margin = 0.03f;
        float obstacle_height_threshold = 0.02f;
        float max_detection_distance = 2.0f;
        float fx = 617.0f, fy = 617.0f;
        float cx = 320.0f, cy = 240.0f;
    };

    ObstacleDetector(const PlaneEstimator& estimator, const Parameters& params);
    int detect(const cv::Mat& depth, cv::Mat& debug_img, float scale);
    int get_ground_row() const { return ground_row_; }

private:
    const PlaneEstimator& plane_estimator_;  // 添加成员
    Parameters params_;
    int ground_row_;
    cv::Mat calculate_height_map(const cv::Mat& depth, float scale) const;
};

} // namespace obs_filiter