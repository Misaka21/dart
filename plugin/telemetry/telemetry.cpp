#include "telemetry.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#ifdef ENABLE_WEBVIEW
#include <pybind11/embed.h>
#endif

#include "plugin/debug/logger.hpp"
#include "plugin/param/thirdparty/toml.hpp"

namespace telemetry {
namespace {

struct Sample {
    uint64_t seq = 0;
    SeriesId id = INVALID_SERIES_ID;
    int64_t t_us = 0;
    float value = 0.0f;
};

struct SeriesInfo {
    SeriesId id = INVALID_SERIES_ID;
    std::string path;
};

std::mutex g_mutex;
std::unordered_map<std::string, SeriesId> g_path_to_id;
std::vector<SeriesInfo> g_series;
std::vector<Sample> g_samples;
uint64_t g_next_seq = 1;

std::once_flag g_init_once;
std::atomic<bool> g_enabled{false};
std::atomic<int> g_publish_interval_ms{33};
std::atomic<int> g_max_batch_samples{4096};
std::atomic<double> g_default_time_window_s{10.0};
std::atomic<int> g_max_series{512};

template <typename T>
T read_value(const toml::table& table, const char* table_name, const char* key_name, T fallback) {
    const toml::node* node = table[table_name][key_name].node();
    if (!node) return fallback;

    if constexpr (std::is_same_v<T, bool>) {
        const auto value = node->value<bool>();
        return value.value_or(fallback);
    } else if constexpr (std::is_integral_v<T>) {
        const auto value = node->value<int64_t>();
        return value ? static_cast<T>(*value) : fallback;
    } else if constexpr (std::is_floating_point_v<T>) {
        const auto value = node->value<double>();
        return value ? static_cast<T>(*value) : fallback;
    } else {
        return fallback;
    }
}

void init_once() {
    bool enable = true;
    int64_t sample_capacity = 32768;
    int64_t max_series = 512;
    int64_t publish_hz = 30;
    int64_t max_batch_samples = 4096;
    double time_window_s = 10.0;

    try {
        const auto table = toml::parse_file(CONFIG_DIR "/debugger.toml");
        enable = read_value(table, "Telemetry", "enable", enable);
        sample_capacity = read_value(table, "Telemetry", "sample_capacity", sample_capacity);
        max_series = read_value(table, "Telemetry", "max_series", max_series);
        publish_hz = read_value(table, "Telemetry", "publish_hz", publish_hz);
        max_batch_samples = read_value(table, "Telemetry", "max_batch_samples", max_batch_samples);
        time_window_s = read_value(table, "Telemetry", "default_time_window", time_window_s);
    } catch (const std::exception& e) {
        debug::print(debug::PrintMode::ERROR, "Telemetry", "Failed to parse debugger.toml: {}", e.what());
    }

    sample_capacity = std::clamp<int64_t>(sample_capacity, 1024, 1048576);
    max_series = std::clamp<int64_t>(max_series, 16, 4096);
    publish_hz = std::clamp<int64_t>(publish_hz, 1, 120);
    max_batch_samples = std::clamp<int64_t>(max_batch_samples, 128, 65536);
    time_window_s = std::clamp(time_window_s, 1.0, 120.0);

    {
        std::lock_guard lock(g_mutex);
        g_path_to_id.clear();
        g_series.clear();
        g_samples.assign(static_cast<size_t>(sample_capacity), Sample{});
        g_next_seq = 1;
    }

    g_max_series.store(static_cast<int>(max_series), std::memory_order_relaxed);
    g_publish_interval_ms.store(static_cast<int>(1000 / publish_hz), std::memory_order_relaxed);
    g_max_batch_samples.store(static_cast<int>(max_batch_samples), std::memory_order_relaxed);
    g_default_time_window_s.store(time_window_s, std::memory_order_relaxed);
    g_enabled.store(enable, std::memory_order_release);

    debug::print(debug::PrintMode::INFO, "Telemetry",
                 "enable={}, sample_capacity={}, max_series={}, publish_hz={}",
                 enable, sample_capacity, max_series, publish_hz);
}

void ensure_initialized() {
    std::call_once(g_init_once, init_once);
}

int64_t now_us() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

SeriesId register_series_locked(std::string_view path) {
    const std::string key(path);
    const auto found = g_path_to_id.find(key);
    if (found != g_path_to_id.end()) {
        return found->second;
    }

    if (g_series.size() >= static_cast<size_t>(g_max_series.load(std::memory_order_relaxed))) {
        return INVALID_SERIES_ID;
    }

    const auto id = static_cast<SeriesId>(g_series.size());
    g_path_to_id.emplace(key, id);
    g_series.push_back(SeriesInfo{id, key});
    return id;
}

void write_sample(SeriesId id, double value) {
    ensure_initialized();
    if (!g_enabled.load(std::memory_order_acquire) || id == INVALID_SERIES_ID) {
        return;
    }

    std::lock_guard lock(g_mutex);
    if (g_samples.empty() || id >= g_series.size()) {
        return;
    }

    const uint64_t seq = g_next_seq++;
    auto& slot = g_samples[static_cast<size_t>(seq % g_samples.size())];
    slot.seq = seq;
    slot.id = id;
    slot.t_us = now_us();
    slot.value = static_cast<float>(value);
}

}  // namespace

#ifdef ENABLE_WEBVIEW
namespace py = pybind11;

py::dict poll_py(uint64_t after_seq, size_t max_samples) {
    ensure_initialized();

    const size_t config_max = static_cast<size_t>(g_max_batch_samples.load(std::memory_order_relaxed));
    if (max_samples == 0 || max_samples > config_max) {
        max_samples = config_max;
    }

    py::dict out;
    out["enabled"] = g_enabled.load(std::memory_order_acquire);
    out["publish_interval_ms"] = g_publish_interval_ms.load(std::memory_order_relaxed);
    out["time_window_s"] = g_default_time_window_s.load(std::memory_order_relaxed);

    py::list series;
    py::list samples;
    bool reset = false;
    uint64_t next_seq = after_seq;

    {
        std::lock_guard lock(g_mutex);
        for (const auto& item : g_series) {
            py::list entry;
            entry.append(item.id);
            entry.append(item.path);
            series.append(entry);
        }

        if (!g_samples.empty() && g_next_seq > 1) {
            const uint64_t newest = g_next_seq - 1;
            const uint64_t capacity = static_cast<uint64_t>(g_samples.size());
            const uint64_t oldest = newest >= capacity ? newest - capacity + 1 : 1;
            uint64_t start = after_seq + 1;
            if (start < oldest) {
                start = oldest;
                reset = true;
            }

            if (start <= newest) {
                const uint64_t end = std::min<uint64_t>(newest, start + max_samples - 1);
                for (uint64_t seq = start; seq <= end; ++seq) {
                    const auto& sample = g_samples[static_cast<size_t>(seq % g_samples.size())];
                    if (sample.seq != seq) {
                        continue;
                    }
                    py::list item;
                    item.append(sample.seq);
                    item.append(sample.id);
                    item.append(sample.t_us);
                    item.append(sample.value);
                    samples.append(item);
                }
                next_seq = end;
            }
        }
    }

    out["series"] = series;
    out["samples"] = samples;
    out["next_seq"] = next_seq;
    out["reset"] = reset;
    return out;
}
#endif

void init_from_config() {
    ensure_initialized();
}

void shutdown() {
    g_enabled.store(false, std::memory_order_release);
}

bool enabled() {
    ensure_initialized();
    return g_enabled.load(std::memory_order_acquire);
}

int publish_interval_ms() {
    ensure_initialized();
    return g_publish_interval_ms.load(std::memory_order_relaxed);
}

double default_time_window_s() {
    ensure_initialized();
    return g_default_time_window_s.load(std::memory_order_relaxed);
}

SeriesId series_id(std::string_view path) {
    ensure_initialized();
    if (!g_enabled.load(std::memory_order_acquire)) {
        return INVALID_SERIES_ID;
    }

    std::lock_guard lock(g_mutex);
    return register_series_locked(path);
}

void scalar(SeriesId id, double value) {
    write_sample(id, value);
}

void scalar(SeriesId id, int value) {
    write_sample(id, static_cast<double>(value));
}

void scalar(SeriesId id, bool value) {
    write_sample(id, value ? 1.0 : 0.0);
}

void scalar(std::string_view path, double value) {
    const auto id = series_id(path);
    write_sample(id, value);
}

void scalar(std::string_view path, int value) {
    scalar(path, static_cast<double>(value));
}

void scalar(std::string_view path, bool value) {
    scalar(path, value ? 1.0 : 0.0);
}

Series::Series(std::string_view path)
    : id_(series_id(path)) {
}

void Series::scalar(double value) const {
    telemetry::scalar(id_, value);
}

void Series::scalar(int value) const {
    telemetry::scalar(id_, value);
}

void Series::scalar(bool value) const {
    telemetry::scalar(id_, value);
}

bool Series::valid() const {
    return id_ != INVALID_SERIES_ID;
}

}  // namespace telemetry

#ifdef ENABLE_WEBVIEW
PYBIND11_EMBEDDED_MODULE(Telemetry, m) {
    m.def("poll", &telemetry::poll_py, pybind11::arg("after_seq") = 0, pybind11::arg("max_samples") = 4096);
    m.def("enabled", &telemetry::enabled);
    m.def("publish_interval_ms", &telemetry::publish_interval_ms);
    m.def("default_time_window_s", &telemetry::default_time_window_s);
}
#endif
