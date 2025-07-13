#pragma once
#include <vector>
#include <deque>
#include <opencv2/core.hpp>

namespace obs_filiter {

class PlaneEstimator {
public:
    struct Parameters {
        int ransac_iterations = 500;
        float plane_threshold = 0.02f;
        float normal_constraint = 0.8f;
    };

    explicit PlaneEstimator(const Parameters& params = Parameters());
    bool estimate(const std::vector<cv::Point3f>& points);
    cv::Vec4f get_average_plane() const;

private:
    Parameters params_;
    std::deque<cv::Vec4f> plane_history_;
    cv::Vec4f current_plane_;
    void fit_plane(const std::vector<cv::Point3f>& points, cv::Vec4f& plane);
};

} // namespace obs_filiter