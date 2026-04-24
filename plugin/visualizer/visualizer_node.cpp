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

    const std::string window_name = "RobotCV Detector";
    debug::print(debug::PrintMode::INFO, "VisualizerNode", "Visualizer thread started");

    while (running->get()) {
        bool show_window = runtime_param::find_param("Visualizer.show_window")
            ? runtime_param::get_param<bool>("Visualizer.show_window")
            : false;

        if (!show_window) {
            cv::destroyWindow(window_name);
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
            cv::imshow(window_name, bgr);
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

    cv::destroyWindow(window_name);
    debug::print(debug::PrintMode::INFO, "VisualizerNode", "Visualizer thread stopped");
}

}  // namespace visualizer
