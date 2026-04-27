#ifndef PLUGIN_TELEMETRY_TELEMETRY_HPP
#define PLUGIN_TELEMETRY_TELEMETRY_HPP

#include <cstdint>
#include <string_view>

namespace telemetry {

using SeriesId = uint16_t;
constexpr SeriesId INVALID_SERIES_ID = UINT16_MAX;

void init_from_config();
void shutdown();
bool enabled();
int publish_interval_ms();
double default_time_window_s();

SeriesId series_id(std::string_view path);
void scalar(SeriesId id, double value);
void scalar(SeriesId id, int value);
void scalar(SeriesId id, bool value);

void scalar(std::string_view path, double value);
void scalar(std::string_view path, int value);
void scalar(std::string_view path, bool value);

class Series {
public:
    explicit Series(std::string_view path);

    void scalar(double value) const;
    void scalar(int value) const;
    void scalar(bool value) const;
    bool valid() const;

private:
    SeriesId id_;
};

}  // namespace telemetry

#endif  // PLUGIN_TELEMETRY_TELEMETRY_HPP
