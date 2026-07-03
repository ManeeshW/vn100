// src/vn100.cpp
#include "vn100.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <iostream>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <vector>

using namespace Eigen;
using namespace std::chrono;

const double PI = std::acos(-1.0);

std::string ltrim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    return (start == std::string::npos) ? "" : s.substr(start);
}

std::string rtrim(const std::string& s) {
    size_t end = s.find_last_not_of(" \t\r\n");
    return (end == std::string::npos) ? "" : s.substr(0, end + 1);
}

std::string trim(const std::string& s) {
    return rtrim(ltrim(s));
}

vn100::vn100() : is_open(false), has_data(false), z_packet_counter(0) {
    load_config("");
}

vn100::~vn100() {
    if (is_open) close();
}

void vn100::load_config(const std::string& config_path) {
    // The configuration file uses a simple key=value format.
    // - Lines starting with # are comments.
    // - Whitespace around keys, values, and = is trimmed.
    // - If no file is provided (empty config_path), defaults are used.
    // - If file is provided but not found, defaults are used with a warning.
    //
    // Example config.cfg content:
    // # IMU Configuration File
    // IMU.on = 1                  # 1 to enable, 0 to disable
    // IMU.port = /dev/ttyTHS1     # Serial port
    // IMU.baud_rate = 230400      # Baud rate
    // IMU.port_num = 2            # 1 for PORT1, 2 for PORT2 (USB/FTDI)
    // IMU.mag_declination = 0.0   # Magnetic declination in degrees
    // IMU.frequency = 200         # Output frequency in Hz

    port = "/dev/ttyTHS1";
    baud_rate = 230400;
    port_number = 2;
    on = true;
    frequency = 200;
    zenoh_topic = "fdcl/imu";
    zenoh_publish_rate = 200;
    double mag_deg = 0.0;
    use_butterworth = true;
    gyro_butter_order = 2;
    gyro_butter_cutoff = 30.0;
    accel_butter_order = 2;
    accel_butter_cutoff = 30.0;
    publish_filtered_imu = true;
    filtered_imu_topic = "fdcl/rover_imu_filtered";
    calibrate_bias = true;
    calib_duration_sec = 10.0;
    calib_warmup_sec = 1.0;
    calib_gyro_motion_thresh = 0.02;
    calib_accel_motion_thresh = 0.30;

    bool config_loaded = false;

    if (!config_path.empty()) {
        std::ifstream file(config_path);
        if (!file.is_open()) {
            std::cout << "Config file '" << config_path << "' not found, using defaults." << std::endl;
        } else {
            std::string line;
            while (std::getline(file, line)) {
                line = trim(line);
                if (line.empty() || line[0] == '#') continue;
                size_t eq_pos = line.find('=');
                if (eq_pos == std::string::npos) continue;
                std::string key = trim(line.substr(0, eq_pos));
                std::string val = trim(line.substr(eq_pos + 1));
                try {
                    if (key == "IMU.port") {
                        port = val;
                    } else if (key == "IMU.baud_rate") {
                        baud_rate = std::stoi(val);
                    } else if (key == "IMU.port_num") {
                        port_number = std::stoi(val);
                    } else if (key == "IMU.mag_declination") {
                        mag_deg = std::stod(val);
                    } else if (key == "IMU.on") {
                        on = (std::stoi(val) != 0);
                    } else if (key == "IMU.frequency") {
                        frequency = std::stoi(val);
                    } else if (key == "IMU.zenoh_topic") {
                        zenoh_topic = val;
                    } else if (key == "IMU.zenoh_publish_rate") {
                        zenoh_publish_rate = std::stoi(val);
                    } else if (key == "IMU.use_butterworth") {
                        use_butterworth = (std::stoi(val) != 0);
                    } else if (key == "IMU.gyro_butter_order") {
                        gyro_butter_order = std::stoi(val);
                    } else if (key == "IMU.gyro_butter_cutoff") {
                        gyro_butter_cutoff = std::stod(val);
                    } else if (key == "IMU.accel_butter_order") {
                        accel_butter_order = std::stoi(val);
                    } else if (key == "IMU.accel_butter_cutoff") {
                        accel_butter_cutoff = std::stod(val);
                    } else if (key == "IMU.publish_filtered_imu") {
                        publish_filtered_imu = (std::stoi(val) != 0);
                    } else if (key == "IMU.filtered_imu_topic") {
                        filtered_imu_topic = val;
                    } else if (key == "IMU.calibrate_bias") {
                        calibrate_bias = (std::stoi(val) != 0);
                    } else if (key == "IMU.calib_duration") {
                        calib_duration_sec = std::stod(val);
                    } else if (key == "IMU.calib_warmup") {
                        calib_warmup_sec = std::stod(val);
                    } else if (key == "IMU.calib_gyro_motion_thresh") {
                        calib_gyro_motion_thresh = std::stod(val);
                    } else if (key == "IMU.calib_accel_motion_thresh") {
                        calib_accel_motion_thresh = std::stod(val);
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error parsing config line: " << line << " - " << e.what() << std::endl;
                }
            }
            std::cout << "Config file '" << config_path << "' loaded successfully." << std::endl;
            config_loaded = true;
        }
    } else {
        std::cout << "No config file provided, using defaults." << std::endl;
    }

    mag_declination = mag_deg * PI / 180.0;

    if (zenoh_publish_rate > frequency) {
        std::cerr << "Warning: zenoh_publish_rate (" << zenoh_publish_rate
                  << ") > frequency (" << frequency << "), clamping to frequency." << std::endl;
        zenoh_publish_rate = frequency;
    }

    std::cout << "IMU Config loaded - Port: " << port
              << ", Baud: " << baud_rate
              << ", Port Num: " << port_number
              << ", Frequency: " << frequency
              << ", Mag Declination: " << mag_deg << " deg (rad: " << mag_declination << ")"
              << ", Enabled: " << (on ? "true" : "false")
              << ", Zenoh Topic: " << zenoh_topic
              << ", Zenoh Publish Rate: " << zenoh_publish_rate << " Hz"
              << ", Butterworth: " << (use_butterworth ? "enabled" : "disabled")
              << ", Gyro [order=" << gyro_butter_order << " cutoff=" << gyro_butter_cutoff << " Hz]"
              << ", Accel [order=" << accel_butter_order << " cutoff=" << accel_butter_cutoff << " Hz]"
              << ", Publish Filtered: " << (publish_filtered_imu ? filtered_imu_topic : "no")
              << ", Bias Calib: " << (calibrate_bias ? "enabled" : "disabled")
              << " [warmup=" << calib_warmup_sec << "s duration=" << calib_duration_sec << "s]"
              << std::endl;

    if (config_loaded) {
        std::cout << "Config file was successfully loaded and applied." << std::endl;
    } else {
        std::cout << "Config file was NOT loaded; defaults were used." << std::endl;
    }
}

bool vn100::enabled() const {
    return on;
}

void vn100::open(const std::string& p, int b) {
    std::cout << "IMU: connecting to IMU at " << p << " .." << std::endl;
    vs.connect(p, b);
    std::string model_num = vs.readModelNumber();
    std::cout << "IMU: connected to " << model_num << std::endl;
    is_open = true;

    // Initialize Butterworth filters
    if (use_butterworth) {
        gyro_filter  = ButterworthFilter(gyro_butter_order,  gyro_butter_cutoff,  static_cast<double>(frequency), 3);
        accel_filter = ButterworthFilter(accel_butter_order, accel_butter_cutoff, static_cast<double>(frequency), 3);
        std::cout << "IMU: Butterworth filters initialized." << std::endl;
    }

    // Initialize startup bias calibration state
    gyro_sum.setZero();   accel_sum.setZero();
    gyro_sumsq.setZero(); accel_sumsq.setZero();
    gyro_bias.setZero();  accel_bias.setZero();
    calib_count = 0;
    if (calibrate_bias) {
        calib_warmup_samples = std::max(0,   static_cast<int>(calib_warmup_sec   * frequency));
        calib_target_samples = std::max(1,   static_cast<int>(calib_duration_sec * frequency));
        calib_state = (calib_warmup_samples > 0) ? CalibState::Warmup : CalibState::Collecting;
        std::cout << "IMU: stationary bias calibration enabled - keep the IMU still for the first "
                  << (calib_warmup_sec + calib_duration_sec) << " s." << std::endl;
    } else {
        calib_state = CalibState::Done;  // bias stays zero
        std::cout << "IMU: bias calibration disabled." << std::endl;
    }

    std::cout << "IMU: initializing Zenoh session on topic '" << zenoh_topic << "' at "
              << zenoh_publish_rate << " Hz .." << std::endl;
    try {
        z_session.emplace(zenoh::Session::open(zenoh::Config::create_default()));
        z_publisher.emplace(z_session->declare_publisher(zenoh_topic));
        z_packet_counter = 0;
        std::cout << "IMU: Zenoh publisher ready." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "IMU: Zenoh init failed: " << e.what()
                  << " - publishing disabled." << std::endl;
        z_publisher.reset();
        z_session.reset();
    }

    // Initialize filtered IMU publisher if enabled
    if (publish_filtered_imu) {
        try {
            z_filtered_session.emplace(zenoh::Session::open(zenoh::Config::create_default()));
            z_filtered_publisher.emplace(z_filtered_session->declare_publisher(filtered_imu_topic));
            std::cout << "IMU: Filtered IMU Zenoh publisher ready on '" << filtered_imu_topic << "'." << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "IMU: Filtered Zenoh init failed: " << e.what()
                      << " - filtered publishing disabled." << std::endl;
            z_filtered_publisher.reset();
            z_filtered_session.reset();
        }
    }

    std::cout << "IMU: setting up binary output register .." << std::endl;
    vs.writeAsyncDataOutputType(VNOFF);  // disable ASCII output

    int divider = 800 / frequency;
    config_vn100_message(divider);

    std::cout << "IMU: starting sleep...\n" << std::endl;
}

void vn100::open() {
    if (!on) {
        throw std::runtime_error("IMU is disabled in config.");
    }
    open(port, baud_rate);
}

void vn100::close() {
    std::cout << "IMU: closing sensor .." << std::endl;
    vs.unregisterAsyncPacketReceivedHandler();

    if (z_publisher.has_value()) {
        z_publisher.reset();
        std::cout << "IMU: Zenoh publisher closed." << std::endl;
    }
    if (z_session.has_value()) {
        z_session.reset();
        std::cout << "IMU: Zenoh session closed." << std::endl;
    }

    if (z_filtered_publisher.has_value()) {
        z_filtered_publisher.reset();
        std::cout << "IMU: Filtered Zenoh publisher closed." << std::endl;
    }
    if (z_filtered_session.has_value()) {
        z_filtered_session.reset();
        std::cout << "IMU: Filtered Zenoh session closed." << std::endl;
    }

    int divider = 0;
    vn::protocol::uart::AsyncMode port_mode;
    if (port_number == 1) {
        port_mode = ASYNCMODE_PORT1;
    } else if (port_number == 2) {
        port_mode = ASYNCMODE_PORT2;
    } else {
        std::cerr << "IMU: invalid port number " << port_number << ", defaulting to PORT2" << std::endl;
        port_mode = ASYNCMODE_PORT2;
    }

    vn::sensors::BinaryOutputRegister bor(
        port_mode,
        divider,
        COMMONGROUP_NONE,
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        GPSGROUP_NONE,
        ATTITUDEGROUP_NONE,
        INSGROUP_NONE);
    vs.writeBinaryOutput1(bor);
   
    std::cout << "IMU: disconnecting sensor .." << std::endl;
    vs.disconnect();

    is_open = false;
}

void vn100::config_vn100_message(const int divider) {
    vn::protocol::uart::AsyncMode port_mode;
    if (port_number == 1) {
        port_mode = ASYNCMODE_PORT1;
    } else if (port_number == 2) {
        port_mode = ASYNCMODE_PORT2;
    } else {
        std::cerr << "IMU: invalid port number " << port_number << ", defaulting to PORT2" << std::endl;
        port_mode = ASYNCMODE_PORT2;
    }

    vn::sensors::BinaryOutputRegister bor(
        port_mode,
        divider,
        COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE,
        TIMEGROUP_NONE,
        IMUGROUP_NONE,
        GPSGROUP_NONE,
        ATTITUDEGROUP_LINEARACCELBODY,
        INSGROUP_NONE);

    vs.writeBinaryOutput1(bor);

    vs.registerAsyncPacketReceivedHandler(this, parse_vn100_packet);
}

void vn100::parse_vn100_packet(void* userData, Packet& p, size_t index) {
    vn100* self = static_cast<vn100*>(userData);
    if (!self) return;

    if (p.type() != Packet::TYPE_BINARY) {
        return;
    }
    
    check_binary_message_vn100(p);

    vec3f ypr_deg = p.extractVec3f();
    vec4f quat = p.extractVec4f();
    vec3f ang_rate = p.extractVec3f();
    vec3f accel = p.extractVec3f();

    Vector3d YPR_rad;
    YPR_rad(0) = ypr_deg[0] * PI / 180.0 - self->mag_declination;
    YPR_rad(1) = ypr_deg[1] * PI / 180.0;
    YPR_rad(2) = ypr_deg[2] * PI / 180.0;

    Eigen::VectorXd W_i(3), a_i(3);
    W_i << ang_rate[0], ang_rate[1], ang_rate[2];
    a_i << accel[0], accel[1], accel[2];

    // ---- Startup stationary bias calibration ----
    // The IMU is assumed at rest at startup. Discard a warmup window, then
    // average ang_rate and accel to estimate the constant bias and remove it.
    if (self->calib_state != CalibState::Done) {
        if (self->calib_state == CalibState::Warmup) {
            if (++self->calib_count >= self->calib_warmup_samples) {
                self->calib_state = CalibState::Collecting;
                self->calib_count = 0;
                self->gyro_sum.setZero();   self->accel_sum.setZero();
                self->gyro_sumsq.setZero(); self->accel_sumsq.setZero();
                std::cout << "\nIMU: warmup complete, collecting bias samples "
                             "(keep IMU stationary)..." << std::endl;
            }
        } else {  // Collecting
            self->gyro_sum    += W_i;
            self->accel_sum   += a_i;
            self->gyro_sumsq  += W_i.cwiseProduct(W_i);
            self->accel_sumsq += a_i.cwiseProduct(a_i);
            if (++self->calib_count >= self->calib_target_samples) {
                const double n = static_cast<double>(self->calib_count);
                self->gyro_bias  = self->gyro_sum  / n;
                self->accel_bias = self->accel_sum / n;
                Eigen::Vector3d gyro_std =
                    ((self->gyro_sumsq  / n) - self->gyro_bias.cwiseProduct(self->gyro_bias))
                        .cwiseMax(0.0).cwiseSqrt();
                Eigen::Vector3d accel_std =
                    ((self->accel_sumsq / n) - self->accel_bias.cwiseProduct(self->accel_bias))
                        .cwiseMax(0.0).cwiseSqrt();
                bool moved = (gyro_std.maxCoeff()  > self->calib_gyro_motion_thresh) ||
                             (accel_std.maxCoeff() > self->calib_accel_motion_thresh);
                self->calib_state = CalibState::Done;

                // Reset filters so the bias step change doesn't create a transient.
                if (self->use_butterworth) {
                    self->gyro_filter  = ButterworthFilter(
                        self->gyro_butter_order,  self->gyro_butter_cutoff,
                        static_cast<double>(self->frequency), 3);
                    self->accel_filter = ButterworthFilter(
                        self->accel_butter_order, self->accel_butter_cutoff,
                        static_cast<double>(self->frequency), 3);
                }

                std::cout << std::fixed << std::setprecision(6)
                          << "\nIMU: bias calibration complete (" << self->calib_count << " samples)\n"
                          << "  gyro  bias [rad/s] : " << self->gyro_bias(0)  << "  "
                          << self->gyro_bias(1)  << "  " << self->gyro_bias(2)
                          << "   (std " << gyro_std(0) << " " << gyro_std(1) << " " << gyro_std(2) << ")\n"
                          << "  accel bias [m/s^2] : " << self->accel_bias(0) << "  "
                          << self->accel_bias(1) << "  " << self->accel_bias(2)
                          << "   (std " << accel_std(0) << " " << accel_std(1) << " " << accel_std(2) << ")"
                          << std::endl;
                if (moved) {
                    std::cerr << "IMU: WARNING - motion detected during calibration "
                                 "(std above threshold). Bias may be inaccurate; "
                                 "keep the IMU stationary and restart to recalibrate."
                              << std::endl;
                }
            }
        }
        // bias is zero until calibration completes (robot is stationary anyway)
    }

    // Remove estimated bias so published values are zero-mean at rest.
    W_i -= self->gyro_bias;
    a_i -= self->accel_bias;
    ang_rate[0] = static_cast<float>(W_i(0));
    ang_rate[1] = static_cast<float>(W_i(1));
    ang_rate[2] = static_cast<float>(W_i(2));
    accel[0] = static_cast<float>(a_i(0));
    accel[1] = static_cast<float>(a_i(1));
    accel[2] = static_cast<float>(a_i(2));

    // Apply Butterworth filters if enabled
    Eigen::VectorXd W_filt = W_i;
    Eigen::VectorXd a_filt = a_i;
    if (self->use_butterworth) {
        W_filt = self->gyro_filter.update(W_i);
        a_filt = self->accel_filter.update(a_i);
    }

    double cy = cos(YPR_rad(0));
    double sy = sin(YPR_rad(0));
    double cp = cos(YPR_rad(1));
    double sp = sin(YPR_rad(1));
    double cr = cos(YPR_rad(2));
    double sr = sin(YPR_rad(2));

    Matrix3d R_ni;
    R_ni(0, 0) = cy * cp;
    R_ni(0, 1) = cy * sr * sp - cr * sy;
    R_ni(0, 2) = sy * sr + cy * cr * sp;
    R_ni(1, 0) = cp * sy;
    R_ni(1, 1) = cy * cr + sy * sr * sp;
    R_ni(1, 2) = cr * sy * sp - cy * sr;
    R_ni(2, 0) = -sp;
    R_ni(2, 1) = cp * sr;
    R_ni(2, 2) = cr * cp;

    double decl_deg = self->mag_declination * 180.0 / PI;
    double adjusted_yaw_deg = ypr_deg[0] - decl_deg;

    {
        std::lock_guard<std::mutex> lock(self->data_mutex);
        self->latest_ypr_deg = ypr_deg;
        self->latest_ang_rate = ang_rate;
        self->latest_quat = quat;
        self->latest_accel = accel;
        self->has_data = true;
    }

    if (self->z_publisher.has_value()) {
        int skip = (self->frequency > 0 && self->zenoh_publish_rate > 0)
                   ? (self->frequency / self->zenoh_publish_rate) : 1;
        if (skip < 1) skip = 1;

        self->z_packet_counter++;
        if (self->z_packet_counter >= skip) {
            self->z_packet_counter = 0;

            auto ts = duration_cast<duration<double>>(
                system_clock::now().time_since_epoch()).count();

            char buf[768];
            std::snprintf(buf, sizeof(buf),
                "{\"timestamp\":%.6f,"
                "\"ypr_deg\":[%.4f,%.4f,%.4f],"
                "\"quat\":[%.6f,%.6f,%.6f,%.6f],"
                "\"ang_rate\":[%.6f,%.6f,%.6f],"
                "\"accel\":[%.6f,%.6f,%.6f],"
                "\"ang_rate_filtered\":[%.6f,%.6f,%.6f],"
                "\"accel_filtered\":[%.6f,%.6f,%.6f]}",
                ts,
                ypr_deg[0], ypr_deg[1], ypr_deg[2],
                quat[0], quat[1], quat[2], quat[3],
                ang_rate[0], ang_rate[1], ang_rate[2],
                accel[0], accel[1], accel[2],
                W_filt(0), W_filt(1), W_filt(2),
                a_filt(0), a_filt(1), a_filt(2));

            self->z_publisher->put(zenoh::Bytes(buf));
        }
    }

    // Publish filtered IMU on separate topic if enabled.
    // Binary wire format: 10 doubles (80 bytes), little-endian
    //   [unused(3), ang_rate(3), accel(3), timestamp(1)]
    // matching ikf_node's parseImu() in zenoh_utils.hpp.
    if (self->z_filtered_publisher.has_value()) {
        auto ts = duration_cast<duration<double>>(
            system_clock::now().time_since_epoch()).count();

        double vals[10] = {
            0.0, 0.0, 0.0,                      // unused
            W_filt(0), W_filt(1), W_filt(2),    // filtered ang_rate
            a_filt(0), a_filt(1), a_filt(2),    // filtered accel
            ts                                  // timestamp
        };

        std::vector<uint8_t> fbuf(sizeof(vals));
        std::memcpy(fbuf.data(), vals, sizeof(vals));
        self->z_filtered_publisher->put(zenoh::Bytes(std::move(fbuf)));
    }

    static int packet_count = 0;
    static steady_clock::time_point start_time = steady_clock::now();
    packet_count++;
    auto current_time = steady_clock::now();
    duration<double> elapsed = current_time - start_time;

    if (elapsed.count() >= 0.1) {
        double hz = packet_count / elapsed.count();
        static bool first_print = true;
        if (!first_print)
            std::cout << "\033[5A";  // move cursor up 5 lines to overwrite
        std::cout << std::fixed
                  << "\033[2K--- IMU @ " << std::setprecision(1) << hz << " Hz ---\n"
                  << std::setprecision(3)
                  << "\033[2K  YPR (deg) : " << ypr_deg[0] << "  " << ypr_deg[1] << "  " << ypr_deg[2] << "\n"
                  << "\033[2K  Quat      : " << quat[0] << "  " << quat[1] << "  " << quat[2] << "  " << quat[3] << "\n"
                  << "\033[2K  AngRate   : " << ang_rate[0] << "  " << ang_rate[1] << "  " << ang_rate[2] << "\n"
                  << "\033[2K  Accel     : " << accel[0] << "  " << accel[1] << "  " << accel[2] << "\n";
        std::cout.flush();
        first_print = false;
        packet_count = 0;
        start_time = current_time;
    }
}

void vn100::check_binary_message_vn100(Packet &p) {
    if (!p.isCompatible(
            COMMONGROUP_QUATERNION | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_ANGULARRATE,
            TIMEGROUP_NONE,
            IMUGROUP_NONE,
            GPSGROUP_NONE,
            ATTITUDEGROUP_LINEARACCELBODY,
            INSGROUP_NONE
            )
        ) {
        std::cout << "Wrong IMU binary packet... \n"
            << "not expecting this!"
            << std::endl;
        return;
    }
}

void vn100::loop() {
    Thread::sleepMs(1);
}

bool vn100::get_latest_data(vec3f& ypr_deg, vec3f& ang_rate, vec4f& quat, vec3f& accel) {
    std::lock_guard<std::mutex> lock(data_mutex);
    if (!has_data) return false;
    ypr_deg = latest_ypr_deg;
    ang_rate = latest_ang_rate;
    quat = latest_quat;
    accel = latest_accel;
    return true;
}