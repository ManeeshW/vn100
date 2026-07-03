// include/vn100.hpp
#ifndef VN100_HPP
#define VN100_HPP

#include <iostream>
#include <string>
#include <mutex>
#include <optional>

#include "vn/sensors.h"
#include "vn/thread.h"
#include "vn/types.h"
#include "vn/util.h"
#include "vn/vector.h"

#include <zenoh.hxx>
#include <Eigen/Dense>

#include "butterworth.hpp"

using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

class vn100 {
public:
    vn100();
    ~vn100();

    void load_config(const std::string& config_path);
    void open(const std::string& port, int baud_rate);
    void open();
    void close();
    void loop();
    bool enabled() const;
    bool get_latest_data(vec3f& ypr_deg, vec3f& ang_rate, vec4f& quat, vec3f& accel);

    std::string port;
    int baud_rate;
    int frequency;
    std::string zenoh_topic;
    int zenoh_publish_rate;

private:
    void config_vn100_message(const int divider);
    static void parse_vn100_packet(void* userData, Packet& p, size_t index);
    static void check_binary_message_vn100(Packet &p);

    int port_number;
    bool on;
    double mag_declination;
    VnSensor vs;
    bool is_open = false;
    vec3f latest_ypr_deg;
    vec3f latest_ang_rate;
    vec3f latest_accel;
    vec4f latest_quat;
    bool has_data = false;
    std::mutex data_mutex;

    // Raw IMU Zenoh publisher
    std::optional<zenoh::Session> z_session;
    std::optional<zenoh::Publisher> z_publisher;
    int z_packet_counter = 0;

    // Butterworth filter settings
    bool use_butterworth = true;
    int gyro_butter_order = 2;
    double gyro_butter_cutoff = 30.0;
    int accel_butter_order = 2;
    double accel_butter_cutoff = 30.0;
    ButterworthFilter gyro_filter;
    ButterworthFilter accel_filter;

    // Filtered IMU Zenoh publisher (separate topic)
    bool publish_filtered_imu = true;
    std::string filtered_imu_topic = "fdcl/rover_imu_filtered";
    std::optional<zenoh::Session> z_filtered_session;
    std::optional<zenoh::Publisher> z_filtered_publisher;

    // Startup stationary bias calibration.
    // The IMU is assumed to be at rest for the first few seconds. We discard a
    // short warmup window, then average ang_rate and accel to estimate the
    // constant bias and subtract it so the published values are zero-mean.
    // (accel is LINEARACCELBODY, i.e. gravity already removed by the VN100, so
    // zero-mean is the correct target at rest.)
    bool   calibrate_bias = true;
    double calib_duration_sec = 10.0;   // length of the averaging window
    double calib_warmup_sec   = 1.0;    // discarded before averaging starts
    double calib_gyro_motion_thresh  = 0.02;  // rad/s, per-axis std limit
    double calib_accel_motion_thresh = 0.30;  // m/s^2, per-axis std limit

    enum class CalibState { Warmup, Collecting, Done };
    CalibState calib_state = CalibState::Warmup;
    int calib_warmup_samples = 0;
    int calib_target_samples = 0;
    int calib_count = 0;
    Eigen::Vector3d gyro_sum    = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sum   = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_sumsq  = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_sumsq = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_bias   = Eigen::Vector3d::Zero();
    Eigen::Vector3d accel_bias  = Eigen::Vector3d::Zero();
};

#endif