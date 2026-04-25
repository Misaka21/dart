#ifndef __DETECTOR_DETECTOR_HPP__
#define __DETECTOR_DETECTOR_HPP__

#include <opencv2/opencv.hpp>

#include "plugin/debug/logger.hpp"
#include "plugin/param/parameter.hpp"
namespace detector
{
    class BaseDetector
    {
    private:
        /* data */
        cv::Mat _frame_img;
        int _dart_id;
        int _dart_number;
        bool _should_detect;
        double _time;
        struct LightInfo
        {
            std::vector<cv::Point> contours;
            cv::Point2f center;
            double area;
            double diameter_px;
            double distance_m;
            double distance_stddev_m;
            bool is_detected;
            LightInfo()
                : contours(),
                  center(0.0f, 0.0f),
                  area(0.0),
                  diameter_px(0.0),
                  distance_m(0.0),
                  distance_stddev_m(0.0),
                  is_detected(false) {}
        } _light_info;
        int _yaw_diff;
        int get_image_center_x()
        {
            return this->_frame_img.cols / 2;
        }
        int get_light_x()
        {
            return this->_light_info.center.x;
        }
        int _target_x;
        int _light_x;
        void update_light_distance(double diameter_px);

    public:
        BaseDetector(/* args */);
        ~BaseDetector();
        void detect(cv::Mat data);
        void set_should_detect(bool should_detect);
        void update_dart_id(int dart_number);
        int calculate_yaw_diff();
        bool get_detected_state()
        {
            return this->_light_info.is_detected;
        };
        double get_light_distance_m()
        {
            return this->_light_info.distance_m;
        }
        void draw(cv::Mat &output_img);
    };
}
#endif //__DETECTOR_DETECTOR_HPP__
