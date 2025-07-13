#pragma once
#include <opencv2/core.hpp>

namespace obs_filiter {

class Visualizer {
public:
    struct Parameters {
        float info_font_scale;
        int info_font_thickness;
        cv::Scalar ground_color;
        cv::Scalar info_text_color;
        cv::Scalar fps_text_color;
        float ground_alpha;

        Parameters()
            : info_font_scale(0.6f),
              info_font_thickness(2),
              ground_color(0, 100, 0),
              info_text_color(200, 200, 0),
              fps_text_color(255, 255, 0),
              ground_alpha(0.3f) {}
    };

    explicit Visualizer(const Parameters& params = Parameters());

    void draw_ground_layer(cv::Mat& img, int row, float alpha) const;
    void draw_info_text(cv::Mat& img, const cv::Vec4f& plane, int obstacle_count, double fps) const;

    void visualize(cv::Mat& img,
                   int ground_row,
                   int obstacle_count,
                   const cv::Vec4f& plane,
                   double fps,
                   int depth_height,
                   int color_height) const;

private:
    Parameters params_;
};

}  // namespace obs_filiter
