//
// Created by david on 24-9-14.
//
#include <iostream>
#include "plugin/param/parameter.hpp"
#include "plugin/rmcv_bag/recorder_node.hpp"
#include "plugin/telemetry/telemetry.hpp"
#include "hardware/hardware_node.hpp"
#include "detector/detector_node.hpp"
#include "umt/umt.hpp"
#include <opencv2/opencv.hpp>

#include <chrono>
#include <thread>
#include <string>


int main() {
	std::string param_file_name = "param.toml";
	auto app_running = umt::BasicObjManager<bool>::find_or_create("app_running", true);
	umt::BasicObjManager<bool>::find_or_create("match_mode", false);

	// 初始化日志会话
	debug::init_session();
	telemetry::init_from_config();

	//////////////实时参数加载线程////////////////
	fmt::print(fmt::fg(fmt::color::gold), "======================Loading parameters======================\n");

	std::thread([=]() { plugin::parameter_run(param_file_name); }).detach();
	plugin::wait_for_param("ok");

	//////////////Hardware 硬件节点线程////////////////
	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Hardware=======================\n");
	std::thread([]() { hardware::start_hardware_node(); }).detach();

	// 等待硬件就绪
	auto hardware_running = umt::BasicObjManager<bool>::find_or_create("hardware_running", false);
	while (!hardware_running->get()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Detector=======================\n");
	std::thread([]() { detector::detector_run(); }).detach();

	fmt::print(fmt::fg(fmt::color::gold), "========================Loading Recorder=======================\n");
	std::thread([]() { rmcv_bag::start_recorder_node(); }).detach();

	// 硬编码参数（argc=2，argv[0]=程序名，argv[1]=脚本路径）
	const wchar_t *w_argv[] = {
		L"./RobotCV",	  // 程序名（可自定义）
		L"../app/app.py", // 硬编码的脚本路径
		nullptr			  // 结束标志
	};

	// 计算参数数量（argc = 数组长度 - 1）
	int argc = sizeof(w_argv) / sizeof(w_argv[0]) - 1;

	int ret = Py_Main(argc, const_cast<wchar_t **>(w_argv));
	app_running->get() = false;
	telemetry::shutdown();
	return ret;
}
