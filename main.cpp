// C++ system headers
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// Third-party library headers
#include <fmt/color.h>
#include <opencv2/opencv.hpp>

// Project headers
#include "plugin/param/parameter.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "plugin/telemetry/telemetry.hpp"
#include "plugin/visualizer/visualizer_node.hpp"
#include "plugin/watchdog/watchdog_node.hpp"
#include "hardware/hardware_node.hpp"
#include "detector/detector_node.hpp"
#include "umt/umt.hpp"

namespace {

std::atomic<bool> g_shutdown_requested{false};

void signal_handler(int sig) {
	if (g_shutdown_requested.exchange(true)) {
		fmt::print(fmt::fg(fmt::color::red), "\n[WARN] 再次收到信号 {}, 强制退出\n", sig);
		std::_Exit(1);
	}
	fmt::print(fmt::fg(fmt::color::yellow), "\n[INFO] 收到信号 {}, 正在退出...\n", sig);
}

void print_usage(const char* prog_name) {
	fmt::print("用法: {} [选项]\n", prog_name);
	fmt::print("选项:\n");
	fmt::print("  --match, -m         比赛模式，强制内录 raw.mkv 和 imu.csv\n");
	fmt::print("  --log-dir <path>    指定日志目录，由 scripts/watchdog.sh 传入\n");
	fmt::print("  --help, -h          显示帮助信息\n");
}

}  // namespace


int main(int argc, char* argv[]) {
	std::string param_file_name = "param.toml";
	bool match_mode = false;
	std::string log_dir;

	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--match") == 0 || std::strcmp(argv[i], "-m") == 0) {
			match_mode = true;
		} else if (std::strcmp(argv[i], "--log-dir") == 0 && i + 1 < argc) {
			log_dir = argv[++i];
		} else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
			print_usage(argv[0]);
			return 0;
		} else {
			fmt::print(fmt::fg(fmt::color::yellow), "[WARN] 未知参数: {}\n", argv[i]);
		}
	}

	auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
	auto match_mode_flag = umt::BasicObjManager<bool>::find_or_create("match_mode", match_mode);
	match_mode_flag->get() = match_mode;

	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	// 初始化日志会话
	if (!log_dir.empty()) {
		debug::init_session(log_dir);
	} else {
		debug::init_session();
	}
	telemetry::init_from_config();

	if (match_mode) {
		fmt::print(fmt::fg(fmt::color::red),
			"======================================================================\n"
			"                     MATCH MODE - 比赛模式                           \n"
			"                      强制内录已启用                                  \n"
			"======================================================================\n");
	}

	//////////////实时参数加载线程////////////////
	fmt::print(fmt::fg(fmt::color::gold), "======================Loading parameters======================\n");

	std::thread param_thread([=]() { plugin::parameter_run(param_file_name); });
	plugin::wait_for_param("ok");

	//////////////Hardware 硬件节点线程////////////////
	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Hardware=======================\n");
	std::thread hardware_thread([]() { hardware::start_hardware_node(); });

	// 等待硬件就绪
	auto hardware_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
	while (!hardware_running->get() && app_running->get()) {
		if (g_shutdown_requested.load()) {
			app_running->get() = false;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (!app_running->get()) {
		if (param_thread.joinable()) param_thread.join();
		if (hardware_thread.joinable()) hardware_thread.join();
		telemetry::shutdown();
		return 0;
	}

	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Detector=======================\n");
	std::thread detector_thread([]() { detector::detector_run(); });

	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Recorder=======================\n");
	std::thread recorder_thread([]() { rmcv_bag::start_recorder_node(); });

	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Visualizer=====================\n");
	std::thread visualizer_thread([]() { visualizer::start_visualizer_node(); });

	std::string heartbeat_file = debug::get_session_path() + "/heartbeat";
	std::thread watchdog_thread([heartbeat_file]() {
		watchdog::start_watchdog_node(heartbeat_file, 5000, 1000);
	});

	std::thread signal_watch_thread([app_running]() {
		while (app_running->get()) {
			if (g_shutdown_requested.load()) {
				app_running->get() = false;
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	});

	// 硬编码参数（argc=2，argv[0]=程序名，argv[1]=脚本路径）
	const wchar_t *w_argv[] = {
		L"./dart2026",	  // 程序名（可自定义）
		L"../app/app.py", // 硬编码的脚本路径
		nullptr			  // 结束标志
	};

	// 计算参数数量（argc = 数组长度 - 1）
	int py_argc = sizeof(w_argv) / sizeof(w_argv[0]) - 1;

	int ret = Py_Main(py_argc, const_cast<wchar_t **>(w_argv));
	app_running->get() = false;

	if (signal_watch_thread.joinable()) signal_watch_thread.join();
	if (param_thread.joinable()) param_thread.join();
	if (hardware_thread.joinable()) hardware_thread.join();
	if (detector_thread.joinable()) detector_thread.join();
	if (recorder_thread.joinable()) recorder_thread.join();
	if (visualizer_thread.joinable()) visualizer_thread.join();
	if (watchdog_thread.joinable()) watchdog_thread.join();

	telemetry::shutdown();
	return ret;
}
