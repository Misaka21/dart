/**
 * @file calibration.cpp
 * @brief Camera calibration tool for generating Detector camera_fx/camera_fy.
 */

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "hardware/hik_cam/hik_camera.hpp"
#include "plugin/debug/logger.hpp"
#include "plugin/param/static_config.hpp"

namespace {

struct CalibrationOptions {
    int board_width = 11;
    int board_height = 8;
    double square_size_mm = 20.0;
    double preview_scale = 0.5;
    int min_images = 10;
    double light_diameter_m = 0.050;
    std::string save_dir;
};

struct CalibrationState {
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<cv::Mat> captured_images;
    cv::Size image_size;
};

void print_usage(const char* argv0) {
    std::cout
        << "用法: " << argv0 << " [选项]\n\n"
        << "选项:\n"
        << "  --board-width <n>       棋盘格内角点列数，默认 11\n"
        << "  --board-height <n>      棋盘格内角点行数，默认 8\n"
        << "  --square-size-mm <mm>   单个棋盘格边长，默认 20.0\n"
        << "  --preview-scale <s>     预览检测缩放比例，默认 0.5\n"
        << "  --min-images <n>        开始标定所需最少图像数，默认 10\n"
        << "  --light-diameter-m <m>  绿灯真实直径，默认 0.050\n"
        << "  --save-dir <dir>        保存采集图像的目录，默认 log/<session>/calibration_images\n"
        << "  -h, --help              显示帮助\n\n"
        << "交互按键:\n"
        << "  Space  采集当前图像\n"
        << "  c      开始标定并输出 param.toml 配置段\n"
        << "  s      保存已采集图像\n"
        << "  u      撤销上一次采集\n"
        << "  q/Esc  退出\n";
}

int read_int_arg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    }
    return std::stoi(argv[++i]);
}

double read_double_arg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    }
    return std::stod(argv[++i]);
}

std::string read_string_arg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    }
    return argv[++i];
}

bool parse_args(int argc, char** argv, CalibrationOptions& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return false;
        }
        if (arg == "--board-width") {
            options.board_width = read_int_arg(argc, argv, i);
        } else if (arg == "--board-height") {
            options.board_height = read_int_arg(argc, argv, i);
        } else if (arg == "--square-size-mm") {
            options.square_size_mm = read_double_arg(argc, argv, i);
        } else if (arg == "--preview-scale") {
            options.preview_scale = read_double_arg(argc, argv, i);
        } else if (arg == "--min-images") {
            options.min_images = read_int_arg(argc, argv, i);
        } else if (arg == "--light-diameter-m") {
            options.light_diameter_m = read_double_arg(argc, argv, i);
        } else if (arg == "--save-dir") {
            options.save_dir = read_string_arg(argc, argv, i);
        } else {
            throw std::runtime_error("Unknown option: " + arg);
        }
    }

    if (options.board_width <= 0 || options.board_height <= 0) {
        throw std::runtime_error("board width/height must be positive");
    }
    if (options.square_size_mm <= 0.0) {
        throw std::runtime_error("square size must be positive");
    }
    if (options.preview_scale <= 0.0 || options.preview_scale > 1.0) {
        throw std::runtime_error("preview scale must be in (0, 1]");
    }
    if (options.min_images <= 0) {
        throw std::runtime_error("min images must be positive");
    }
    if (options.light_diameter_m <= 0.0) {
        throw std::runtime_error("light diameter must be positive");
    }
    return true;
}

camera::CameraConfig load_camera_config(const toml::table& config) {
    camera::CameraConfig cam_config;
    cam_config.use_camera_sn = static_param::get_param<bool>(config, "Camera", "use_camera_sn");
    cam_config.camera_sn = static_param::get_param<std::string>(config, "Camera", "camera_sn");
    cam_config.use_mfs_config = static_param::get_param<bool>(config, "Camera", "use_config_from_file");

    const std::string mfs_filename = static_param::get_param<std::string>(config, "Camera", "config_file_path");
    cam_config.mfs_config_path = std::string(CONFIG_DIR) + "/" + mfs_filename;
    cam_config.use_runtime_config = static_param::get_param<bool>(config, "Camera", "use_camera_config");

    auto param_table = static_param::get_param_table(config, "Camera.config");
    for (const auto& [key, value] : param_table) {
        std::visit([&](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (!std::is_same_v<T, std::vector<int64_t>>) {
                cam_config.runtime_params.emplace_back(key, camera::CameraParam(v));
            }
        }, value);
    }
    return cam_config;
}

std::vector<cv::Point3f> create_object_points(const CalibrationOptions& options) {
    std::vector<cv::Point3f> corners;
    corners.reserve(static_cast<size_t>(options.board_width * options.board_height));
    for (int y = 0; y < options.board_height; ++y) {
        for (int x = 0; x < options.board_width; ++x) {
            corners.emplace_back(
                static_cast<float>(x * options.square_size_mm),
                static_cast<float>(y * options.square_size_mm),
                0.0f);
        }
    }
    return corners;
}

void to_gray(const cv::Mat& frame, cv::Mat& gray) {
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }
}

bool detect_chessboard_fast(const cv::Mat& frame,
                            const CalibrationOptions& options,
                            std::vector<cv::Point2f>& corners) {
    cv::Mat small;
    cv::resize(frame, small, cv::Size(), options.preview_scale, options.preview_scale);

    cv::Mat gray;
    to_gray(small, gray);

    const cv::Size board_size(options.board_width, options.board_height);
    const bool found = cv::findChessboardCorners(
        gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

    if (found) {
        for (auto& point : corners) {
            point.x = static_cast<float>(point.x / options.preview_scale);
            point.y = static_cast<float>(point.y / options.preview_scale);
        }
    }
    return found;
}

bool detect_chessboard_precise(const cv::Mat& frame,
                               const CalibrationOptions& options,
                               std::vector<cv::Point2f>& corners) {
    cv::Mat gray;
    to_gray(frame, gray);

    const cv::Size board_size(options.board_width, options.board_height);
    const bool found = cv::findChessboardCorners(
        gray, board_size, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.001));
    }
    return found;
}

bool calibrate_camera(const CalibrationState& state,
                      const CalibrationOptions& options,
                      cv::Mat& camera_matrix,
                      cv::Mat& dist_coeffs,
                      std::vector<cv::Mat>& rvecs,
                      std::vector<cv::Mat>& tvecs,
                      double& rms_error) {
    if (static_cast<int>(state.image_points.size()) < options.min_images) {
        std::cerr << "Error: 至少需要 " << options.min_images << " 张图像，当前 "
                  << state.image_points.size() << " 张\n";
        return false;
    }
    if (state.image_size.empty()) {
        std::cerr << "Error: 图像尺寸为空\n";
        return false;
    }

    std::cout << "\n开始标定，使用 " << state.image_points.size() << " 张图像...\n";

    camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    rms_error = cv::calibrateCamera(
        state.object_points, state.image_points, state.image_size,
        camera_matrix, dist_coeffs, rvecs, tvecs, cv::CALIB_FIX_K3);
    return true;
}

void print_param_toml_block(const cv::Mat& camera_matrix, double light_diameter_m) {
    const double fx = camera_matrix.at<double>(0, 0);
    const double fy = camera_matrix.at<double>(1, 1);

    std::cout << "\n# 可直接复制到 asset/param.toml 的 [Detector] 段:\n";
    std::cout << "    # 绿灯直径测距\n";
    std::cout << "    #@double 绿灯真实直径 (m)\n";
    std::cout << "    light_diameter = " << std::fixed << std::setprecision(3) << light_diameter_m << "\n";
    std::cout << "    #@double 相机 x 方向焦距 (px)，需与 detector 输入图像尺寸一致\n";
    std::cout << "    camera_fx = " << std::fixed << std::setprecision(6) << fx << "\n";
    std::cout << "    #@double 相机 y 方向焦距 (px)，需与 detector 输入图像尺寸一致\n";
    std::cout << "    camera_fy = " << std::fixed << std::setprecision(6) << fy << "\n";

    debug::print(debug::PrintMode::INFO, "Calibration", "param.toml:");
    debug::print(debug::PrintMode::INFO, "Calibration", "light_diameter = {:.6f}", light_diameter_m);
    debug::print(debug::PrintMode::INFO, "Calibration", "camera_fx = {:.6f}", fx);
    debug::print(debug::PrintMode::INFO, "Calibration", "camera_fy = {:.6f}", fy);
}

void print_calibration_result(const CalibrationState& state,
                              const CalibrationOptions& options,
                              const cv::Mat& camera_matrix,
                              const cv::Mat& dist_coeffs,
                              double rms_error) {
    std::cout << "\n==================== 标定结果 ====================\n";
    std::cout << "RMS 重投影误差: " << rms_error << " px\n";
    std::cout << "图像尺寸: " << state.image_size.width << " x " << state.image_size.height << "\n";
    std::cout << std::setprecision(16);
    std::cout << "fx: " << camera_matrix.at<double>(0, 0) << "\n";
    std::cout << "fy: " << camera_matrix.at<double>(1, 1) << "\n";
    std::cout << "cx: " << camera_matrix.at<double>(0, 2) << "\n";
    std::cout << "cy: " << camera_matrix.at<double>(1, 2) << "\n";
    std::cout << "distort_coeffs: [ "
              << dist_coeffs.at<double>(0) << ", "
              << dist_coeffs.at<double>(1) << ", "
              << dist_coeffs.at<double>(2) << ", "
              << dist_coeffs.at<double>(3) << ", "
              << dist_coeffs.at<double>(4) << " ]\n";

    print_param_toml_block(camera_matrix, options.light_diameter_m);
    std::cout << "===================================================\n";

    debug::print(debug::PrintMode::INFO, "Calibration", "========== 相机内参标定结果 ==========");
    debug::print(debug::PrintMode::INFO, "Calibration", "RMS 重投影误差: {:.6f} px", rms_error);
    debug::print(debug::PrintMode::INFO, "Calibration", "图像尺寸: {} x {}", state.image_size.width, state.image_size.height);
    debug::print(debug::PrintMode::INFO, "Calibration", "fx = {:.16f}", camera_matrix.at<double>(0, 0));
    debug::print(debug::PrintMode::INFO, "Calibration", "fy = {:.16f}", camera_matrix.at<double>(1, 1));
    debug::print(debug::PrintMode::INFO, "Calibration", "cx = {:.16f}", camera_matrix.at<double>(0, 2));
    debug::print(debug::PrintMode::INFO, "Calibration", "cy = {:.16f}", camera_matrix.at<double>(1, 2));
    debug::print(debug::PrintMode::INFO, "Calibration",
        "distort_coeffs: [ {:.16f}, {:.16f}, {:.16f}, {:.16f}, {:.16f} ]",
        dist_coeffs.at<double>(0), dist_coeffs.at<double>(1), dist_coeffs.at<double>(2),
        dist_coeffs.at<double>(3), dist_coeffs.at<double>(4));
}

void save_captured_images(const CalibrationState& state, const std::string& dir) {
    std::filesystem::create_directories(dir);
    for (size_t i = 0; i < state.captured_images.size(); ++i) {
        cv::imwrite(dir + "/calib_" + std::to_string(i) + ".png", state.captured_images[i]);
    }
    std::cout << "已保存 " << state.captured_images.size() << " 张图像到: " << dir << "\n";
}

void draw_preview(const cv::Mat& frame,
                  const CalibrationOptions& options,
                  bool found,
                  const std::vector<cv::Point2f>& corners,
                  size_t captured_count,
                  cv::Mat& display) {
    cv::resize(frame, display, cv::Size(), 0.5, 0.5);

    if (found) {
        std::vector<cv::Point2f> display_corners;
        display_corners.reserve(corners.size());
        for (const auto& point : corners) {
            display_corners.emplace_back(point.x * 0.5f, point.y * 0.5f);
        }
        cv::drawChessboardCorners(
            display, cv::Size(options.board_width, options.board_height), display_corners, true);
        cv::putText(display, "SPACE to capture", cv::Point(10, 50),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    } else {
        cv::putText(display, "No chessboard", cv::Point(10, 50),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
    }

    cv::putText(display, "Captured: " + std::to_string(captured_count),
        cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
}

}  // namespace

int main(int argc, char** argv) {
    CalibrationOptions options;
    try {
        if (!parse_args(argc, argv, options)) {
            return 0;
        }
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        print_usage(argv[0]);
        return 2;
    }

    std::cout << "\n========== 相机标定工具 ==========\n";
    std::cout << "标定板: " << options.board_width << "x" << options.board_height << " 内角点\n";
    std::cout << "格子边长: " << options.square_size_mm << " mm\n";
    std::cout << "绿灯真实直径: " << options.light_diameter_m << " m\n";
    std::cout << "操作: Space=拍照  c=标定  s=保存  u=撤销  q/Esc=退出\n";
    std::cout << "==================================\n\n";

    debug::init_session("calibration");
    if (options.save_dir.empty()) {
        options.save_dir = debug::get_session_path() + "/calibration_images";
    }

    try {
        const auto config = static_param::parse_file("hardware.toml");
        camera::CameraConfig cam_config = load_camera_config(config);
        camera::HikCam cam(cam_config);
        cam.open();

        std::cout << "相机已打开\n";
        std::cout << "采集图像保存目录: " << options.save_dir << "\n";

        cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
        cv::resizeWindow("Calibration", 1280, 800);

        CalibrationState state;
        const std::vector<cv::Point3f> obj_pts = create_object_points(options);
        std::vector<cv::Point2f> corners;
        bool found = false;

        while (true) {
            cv::Mat& frame = cam.capture();
            if (frame.empty()) {
                continue;
            }

            state.image_size = frame.size();
            found = detect_chessboard_fast(frame, options, corners);

            cv::Mat display;
            draw_preview(frame, options, found, corners, state.image_points.size(), display);
            cv::imshow("Calibration", display);

            const int key = cv::waitKey(1) & 0xFF;
            if (key == 27 || key == 'q') {
                break;
            }

            if (key == ' ' && found) {
                std::cout << "正在精确检测..." << std::flush;

                std::vector<cv::Point2f> precise_corners;
                if (detect_chessboard_precise(frame, options, precise_corners)) {
                    state.image_points.push_back(precise_corners);
                    state.object_points.push_back(obj_pts);
                    state.captured_images.push_back(frame.clone());
                    std::cout << " 已采集 " << state.image_points.size() << " 张\n";
                } else {
                    std::cout << " 失败，请重试\n";
                }
            }

            if (key == 'u' && !state.image_points.empty()) {
                state.image_points.pop_back();
                state.object_points.pop_back();
                state.captured_images.pop_back();
                std::cout << "撤销，剩余 " << state.image_points.size() << " 张\n";
            }

            if (key == 's') {
                save_captured_images(state, options.save_dir);
            }

            if (key == 'c') {
                cv::Mat camera_matrix;
                cv::Mat dist_coeffs;
                std::vector<cv::Mat> rvecs;
                std::vector<cv::Mat> tvecs;
                double rms_error = 0.0;

                if (calibrate_camera(state, options, camera_matrix, dist_coeffs, rvecs, tvecs, rms_error)) {
                    print_calibration_result(state, options, camera_matrix, dist_coeffs, rms_error);
                    save_captured_images(state, options.save_dir);
                }
            }
        }

        cv::destroyAllWindows();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
