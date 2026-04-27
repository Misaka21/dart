#ifndef __DETECTOR_DETECTORNODE_HPP__
#define __DETECTOR_DETECTORNODE_HPP__
#include "detector.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/parameter.hpp"
#include "hardware/hardware_node.hpp"
#include <opencv2/imgcodecs.hpp>
#include <pybind11/numpy.h>
#include <optional>

namespace detector
{
    namespace umt = ::umt;
    using namespace std::chrono;


    void detector_run()
    {
        // 订阅 hardware 模块发布的 sync_frame 消息
        auto frame_subscriber = umt::Subscriber<hardware::SyncFrame>("sync_frame");
        auto debug_publisher = umt::Publisher<cv::Mat>("Detector_Debug_Image");
        //auto detector_publisher = umt::Publisher<cv::Mat>("Detector_Result");

        BaseDetector detector;

        while (true)
        {
            try
            {

                while (true)
                {
                    // 获取同步帧数据
                    auto sync_frame = frame_subscriber.pop();
                    cv::Mat camera_frame = sync_frame.image;

                    detector.set_should_detect(sync_frame.serial_valid && sync_frame.serial_data.should_detect);
                    if (sync_frame.serial_valid)
                    {
                        detector.update_dart_id(sync_frame.serial_data.dart_number);
                    }

                    // 进行检测
                    detector.detect(camera_frame);

                    // 计算偏航差值
                    detector.calculate_yaw_diff();
                    detector.publish_telemetry();


                    cv::Mat debug_img = camera_frame.clone();
                    detector.draw(debug_img);


                    debug_publisher.push(debug_img);

                }
            }
            catch (std::exception &e)
            {
                debug::print(debug::PrintMode::ERROR, "Detector", e.what());
                std::this_thread::sleep_for(500ms);
            }
        }
    }



} // namespace detector

namespace py = pybind11;
// cv::Mat --> numpy.ndarray
py::array_t<uint8_t> cvMat2npArray(const cv::Mat &mat)
{
    py::array_t<uint8_t> array({mat.rows, mat.cols, mat.channels()});
    cv::Mat ref_mat(mat.rows, mat.cols, CV_8UC(mat.channels()), array.mutable_data());
    mat.copyTo(ref_mat);
    return array;
}

// numpy.ndarray --> cv::Mat
cv::Mat npArray2cvMat(const py::array_t<uint8_t> &array)
{
    cv::Mat mat;
    return mat;
}

UMT_EXPORT_MESSAGE_ALIAS(cvMat, cv::Mat, c)
{
    c.def(py::init<cv::Mat>());
    c.def(py::init(&npArray2cvMat));
    c.def("get_nparray", cvMat2npArray);
}
#endif
