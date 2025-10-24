// src/vn100.cpp
#include "vn100.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <cmath>
#include <iostream>
#include <chrono>

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

vn100::vn100() : is_open(false), has_data(false) {
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
    double mag_deg = 0.0;

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

    std::cout << "IMU Config loaded - Port: " << port 
              << ", Baud: " << baud_rate 
              << ", Port Num: " << port_number 
              << ", Frequency: " << frequency
              << ", Mag Declination: " << mag_deg << " deg (rad: " << mag_declination << ")" 
              << ", Enabled: " << (on ? "true" : "false") << std::endl;

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

    Vector3d W_i(ang_rate[0], ang_rate[1], ang_rate[2]);
    Vector3d a_i(accel[0], accel[1], accel[2]);

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

    std::cout << "Original YPR (deg): " << ypr_deg[0] << ", " << ypr_deg[1] << ", " << ypr_deg[2] << std::endl;
    std::cout << "Mag Declination (deg): " << decl_deg << std::endl;
    std::cout << "Adjusted Yaw (deg): " << adjusted_yaw_deg << std::endl;
    std::cout << "Angular Rate: " << ang_rate[0] << ", " << ang_rate[1] << ", " << ang_rate[2] << std::endl;
    std::cout << "Acceleration: " << accel[0] << ", " << accel[1] << ", " << accel[2] << std::endl;
    std::cout << "Rotation Matrix R_ni:\n" << R_ni << std::endl;

    {
        std::lock_guard<std::mutex> lock(self->data_mutex);
        self->latest_ypr_deg = ypr_deg;
        self->latest_ang_rate = ang_rate;
        self->latest_quat = quat;
        self->latest_accel = accel;
        self->has_data = true;
    }

    // Calculate and show print frequency (data flow rate)
    static int packet_count = 0;
    static steady_clock::time_point start_time = steady_clock::now();
    packet_count++;
    auto current_time = steady_clock::now();
    duration<double> elapsed = current_time - start_time;

    // Debug print to see progress
    std::cout << "Debug: Received packet #" << packet_count << ", Elapsed time since start: " << elapsed.count() << " seconds" << std::endl;

    if (elapsed.count() >= 0.5) {  // Reduced threshold to 0.5 seconds to show frequency sooner
        double frequency = packet_count / elapsed.count();
        std::cout << "Data flow frequency: " << frequency << " Hz (packets per second)" << std::endl;
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