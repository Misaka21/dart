#include "detector.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace detector
{
    BaseDetector::BaseDetector(/* args */)
        : _dart_id(0), _time(0.0), _yaw_diff(0), _target_x(0), _light_x(0)
    {
    }
    BaseDetector::~BaseDetector()
    {
    }

    void BaseDetector::detect(cv::Mat data)
    {
        //auto [frame_img, _, time] = data;
        this->_frame_img = data;
        this->_light_info = LightInfo();
        cv::Mat green_channel, binary;
        std::vector<cv::Mat> channels;

        // 1. 提取绿色通道（BGR 格式下，通道顺序是 B=0, G=1, R=2）
        cv::split(data, channels);
        green_channel = channels[1];

        double threshold_value = plugin::get_param<int64_t>("Detector.Threshold");
        cv::threshold(green_channel, binary, threshold_value, 255, cv::THRESH_BINARY);
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 设置面积筛选阈值
        int64_t min_area = plugin::get_param<int64_t>("Detector.MinArea"); // 最小面积阈值
        int64_t max_area = plugin::get_param<int64_t>("Detector.MaxArea"); // 最大面积阈值

        // 遍历所有轮廓
        for (size_t i = 0; i < contours.size(); i++)
        {
            // 计算轮廓面积
            double area = contourArea(contours[i]);

            // 根据面积筛选
            if (area > min_area && area < max_area)
            {
                // 计算轮廓的最小外接圆
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contours[i], center, radius);

                // 计算周长
                double perimeter = arcLength(contours[i], true);
                if (perimeter <= 0.0)
                {
                    continue;
                }

                // 计算圆形度 (4π * 面积 / 周长^2)
                double circularity = 4 * CV_PI * area / (perimeter * perimeter);

                // 如果轮廓接近圆形
                if (circularity > 0.7)
                {
                    if (this->_light_info.is_detected && area <= this->_light_info.area)
                    {
                        continue;
                    }

                    const double diameter_px = 2.0 * std::sqrt(area / CV_PI);

                    this->_light_info.area = area;
                    this->_light_info.center = center;
                    this->_light_info.contours = contours[i];
                    this->_light_info.diameter_px = diameter_px;
                    this->_light_info.is_detected = true;
                    this->update_light_distance(diameter_px);
                    // 圆形度阈值，可以根据实际情况调整
                    // 绘制轮廓
                    // cv::drawContours(result, contours, (int)i, cv::Scalar(0, 255, 0), 2);

                    // // 绘制中心点
                    // cv::circle(result, centroid, 5, cv::Scalar(0, 0, 255), -1);

                    // // 绘制最小外接圆
                    // cv::circle(result, center, (int)radius, cv::Scalar(255, 0, 0), 2);

                    // // 添加文本信息
                    // std::string info = "Area: " + std::to_string((int)area) +
                    //                    ", Circ: " + std::to_string(circularity).substr(0, 5);
                    // cv::putText(result, info, cv::Point(centroid.x - 50, centroid.y + 30),
                    //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);

                    // std::cout << "轮廓 #" << i << " - 面积: " << area
                    //           << ", 圆形度: " << circularity
                    //           << ", 中心点: (" << centroid.x << ", " << centroid.y << ")" << std::endl;
                }                    
            }
        }
    }

    void BaseDetector::update_light_distance(double diameter_px)
    {
        const double light_diameter_m = plugin::get_param<double>("Detector.LightDiameter");
        const double fx = plugin::get_param<double>("Detector.CameraFx");
        const double fy = plugin::get_param<double>("Detector.CameraFy");
        const double pixel_sigma = plugin::get_param<double>("Detector.PixelSigma");

        double focal_px = 0.0;
        if (fx > 0.0 && fy > 0.0)
        {
            focal_px = 0.5 * (fx + fy);
        }
        else if (fx > 0.0)
        {
            focal_px = fx;
        }
        else if (fy > 0.0)
        {
            focal_px = fy;
        }

        if (diameter_px <= 0.0 || light_diameter_m <= 0.0 || focal_px <= 0.0)
        {
            this->_light_info.distance_m = 0.0;
            this->_light_info.distance_stddev_m = 0.0;
            return;
        }

        this->_light_info.distance_m = focal_px * light_diameter_m / diameter_px;
        this->_light_info.distance_stddev_m =
            pixel_sigma > 0.0
                ? (this->_light_info.distance_m * this->_light_info.distance_m * pixel_sigma) / (focal_px * light_diameter_m)
                : 0.0;
    }

    int BaseDetector::calculate_yaw_diff()
    {
        this->_target_x = this->get_image_center_x() + plugin::get_param<int64_t>("Detector.yaw_offset." + std::to_string(this->_dart_id));
        this->_light_x = this->get_light_x();
        // 灯在线右侧为正
        return this->get_detected_state() ? this->_light_x - this->_target_x : 0;
    }

    void BaseDetector::draw(cv::Mat &output_img)
    {
        // 如果检测到光源，绘制光源信息
        if (this->get_detected_state())
        {
            // 绘制光源轮廓
            cv::drawContours(output_img, std::vector<std::vector<cv::Point>>{this->_light_info.contours},
                             0, cv::Scalar(0, 0, 255), 2);

            // 绘制光源中心点
            cv::circle(output_img, this->_light_info.center, 5, cv::Scalar(0, 0, 255), -1);

            // 添加面积信息
            std::string area_info = "area: " + std::to_string(static_cast<int>(this->_light_info.area));
            cv::putText(output_img, area_info,
                        cv::Point(this->_light_info.center.x + 50, this->_light_info.center.y - 50),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 255), 2);
            std::ostringstream diameter_stream;
            diameter_stream << "diameter: " << std::fixed << std::setprecision(1) << this->_light_info.diameter_px << " px";
            cv::putText(output_img, diameter_stream.str(),
                        cv::Point(this->_light_info.center.x + 50, this->_light_info.center.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);

            std::ostringstream distance_stream;
            distance_stream << "distance: " << std::fixed << std::setprecision(2) << this->_light_info.distance_m << " m";
            if (this->_light_info.distance_stddev_m > 0.0)
            {
                distance_stream << " +/- " << std::setprecision(2) << this->_light_info.distance_stddev_m << " m";
            }
            cv::putText(output_img, distance_stream.str(),
                        cv::Point(this->_light_info.center.x + 50, this->_light_info.center.y + 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
            // 添加面积信息
            std::string point_info = 
            "(x,y):("+ 
            std::to_string(static_cast<int>(this->_light_info.center.x))
            +","+
            std::to_string(static_cast<int>(this->_light_info.center.y))
            +")";
            cv::putText(output_img, point_info,
                        cv::Point(this->_light_info.center.x + 50, this->_light_info.center.y + 70),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
            // 添加偏航差值信息
            std::string yaw_diff_text = "yaw_diff: " + std::to_string(this->calculate_yaw_diff());
            cv::putText(output_img, yaw_diff_text,
                        cv::Point(this->_light_info.center.x + 50, this->_light_info.center.y + 110),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
        }

        // 在目标位置绘制垂直线
        cv::line(output_img, cv::Point(this->_target_x, 0),
                 cv::Point(this->_target_x, output_img.rows),
                 cv::Scalar(255, 0, 0), 2);

        // 添加目标线标签
        std::string target_position_text = ",target_position: " + std::to_string(this->_target_x);
        std::string dart_id = "dart_id: " + std::to_string(this->_dart_id);
        cv::putText(output_img, dart_id+target_position_text,
                    cv::Point(this->_target_x + 10, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), 2);

        // 如果检测到光源，绘制从光源位置到目标位置的箭头
        if (this->_light_info.is_detected)
        {
            // 绘制从光源到目标的水平线
            int arrow_y = this->_light_info.center.y;
            cv::line(output_img, cv::Point(this->_light_x, arrow_y),
                     cv::Point(this->_target_x, arrow_y),
                     cv::Scalar(0, 255, 255), 2);

            // 绘制箭头头部
            int arrow_direction = (this->_target_x > this->_light_x) ? 1 : -1;
            int arrowhead_length = 10;
            cv::line(output_img, cv::Point(this->_target_x, arrow_y),
                     cv::Point(this->_target_x - arrow_direction * arrowhead_length, arrow_y - arrowhead_length),
                     cv::Scalar(0, 255, 255), 2);
            cv::line(output_img, cv::Point(this->_target_x, arrow_y),
                     cv::Point(this->_target_x - arrow_direction * arrowhead_length, arrow_y + arrowhead_length),
                     cv::Scalar(0, 255, 255), 2);

            // 添加距离标签
            // std::string distance_text = "distance: " + std::to_string(this->calculate_yaw_diff()) + " px";
            // cv::putText(output_img, distance_text,
            //             cv::Point((this->_light_x + this->_target_x) / 2, arrow_y - 10),
            //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);


        }

        // 绘制检测状态信息
        std::string status = this->_light_info.is_detected ? "light_detected" : "light_not_detected";
        cv::putText(output_img, status,
                    cv::Point(10, output_img.rows - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
    }
}
