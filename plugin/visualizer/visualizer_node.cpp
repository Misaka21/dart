#include "visualizer_node.hpp"

#include <chrono>
#include <thread>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "plugin/debug/logger.hpp"
#include "plugin/param/runtime_parameter.hpp"
#include "umt/umt.hpp"

namespace visualizer {

void start_visualizer_node() {
    auto running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
    umt::Subscriber<cv::Mat> debug_sub("Detector_Debug_Image", 1);
    auto tx_debug = umt::BasicObjManager<std::string>::find("serial_tx_debug");
    auto rx_debug = umt::BasicObjManager<std::string>::find("serial_rx_debug");

    const std::string window_name = "dart2026 Detector";
    bool window_opened = false;
    debug::print(debug::PrintMode::INFO, "VisualizerNode", "Visualizer thread started");

    while (running->get()) {
        bool show_window = runtime_param::find_param("Visualizer.show_window")
            ? runtime_param::get_param<bool>("Visualizer.show_window")
            : false;

        if (!show_window) {
            if (window_opened) {
                cv::destroyWindow(window_name);
                window_opened = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        try {
            cv::Mat img = debug_sub.pop_for(50);
            if (img.empty()) {
                cv::waitKey(1);
                continue;
            }

            cv::Mat bgr;
            if (img.channels() == 3) {
                cv::cvtColor(img, bgr, cv::COLOR_RGB2BGR);
            } else {
                bgr = img;
            }

            // 叠加串口收发调试信息
            const int font = cv::FONT_HERSHEY_SIMPLEX;
            const double scale = 0.5;
            const int thickness = 1;
            const int line_h = 18;
            int y_pos = 18;

            if (tx_debug) {
                cv::putText(bgr, tx_debug->load(), cv::Point(8, y_pos),
                    font, scale, cv::Scalar(0, 255, 255), thickness);
                y_pos += line_h;
            }
            if (rx_debug) {
                cv::putText(bgr, rx_debug->load(), cv::Point(8, y_pos),
                    font, scale, cv::Scalar(0, 255, 0), thickness);
            }

            cv::imshow(window_name, bgr);
            window_opened = true;
            cv::waitKey(1);
        } catch (const umt::MessageError_Timeout&) {
            cv::waitKey(1);
        } catch (const umt::MessageError_Stopped&) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } catch (const std::exception& e) {
            debug::print(debug::PrintMode::ERROR, "VisualizerNode", "{}", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    if (window_opened) {
        cv::destroyWindow(window_name);
    }
    debug::print(debug::PrintMode::INFO, "VisualizerNode", "Visualizer thread stopped");
}

}  // namespace visualizer
