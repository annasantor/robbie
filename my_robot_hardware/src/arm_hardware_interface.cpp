#include <sys/ioctl.h>
#include "my_robot_hardware/arm_hardware_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <string>
#include <sstream>
#include <thread>        
#include <chrono>        
#include "rclcpp/rclcpp.hpp"

namespace my_robot_controller
{

// Static member definition
const std::unordered_map<std::string, double> ArmHardwareInterface::JOINT_ZERO_POSITIONS = {
    {"wrist_roll",    140.0},
    {"wrist_pitch",   230.0},
    {"elbow_roll",    145.0},
    {"elbow_pitch",   100.0},
    {"shoulder_roll", 130.0},
    {"shoulder_pitch",110.0},
    {"fingers",       160.0},  // add
    {"thumb",          30.0},  // add
};
// These are examples — adjust to match your actual joint limits from URDF
const std::unordered_map<std::string, double> JOINT_RANGES_RAD = {
    {"shoulder_roll",  M_PI},      // -90° to +90° = π rad total
    {"shoulder_pitch", M_PI},
    {"elbow_roll",     M_PI},
    {"elbow_pitch",    M_PI},
    {"wrist_roll",     M_PI},
    {"wrist_pitch",    M_PI},
};
const std::vector<std::string> ArmHardwareInterface::ARDUINO_ORDER = {
    "wrist_roll", "wrist_pitch", "elbow_roll",
    "elbow_pitch", "shoulder_roll", "shoulder_pitch"
};


ArmHardwareInterface::ArmHardwareInterface()
{
}

ArmHardwareInterface::~ArmHardwareInterface()
{
    if (serial_fd_ != -1) {
        close(serial_fd_);
        serial_fd_ = -1;
    }
}

// on_init 

CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    // -- Serial parameters (strict: missing = fatal) --
    if (!params.hardware_info.hardware_parameters.count("serial_port")) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
            "Missing required hardware parameter: serial_port");
        return CallbackReturn::ERROR;
    }
    if (!params.hardware_info.hardware_parameters.count("baud_rate")) {
        RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
            "Missing required hardware parameter: baud_rate");
        return CallbackReturn::ERROR;
    }
    serial_port_ = params.hardware_info.hardware_parameters.at("serial_port");
    baud_rate_   = std::stoi(params.hardware_info.hardware_parameters.at("baud_rate"));
    use_serial_  = (params.hardware_info.hardware_parameters.count("use_serial") &&
                    params.hardware_info.hardware_parameters.at("use_serial") == "true");

    // -- Joint metadata from URDF --
    for (const auto & joint : params.hardware_info.joints) {

        if (JOINT_ZERO_POSITIONS.find(joint.name) == JOINT_ZERO_POSITIONS.end()) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "Joint '%s' not found in JOINT_ZERO_POSITIONS", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.command_interfaces.size() != 1 ||
            joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "Joint '%s' must have exactly one position command interface", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
        if (joint.state_interfaces.size() != 1 ||
            joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "Joint '%s' must have exactly one position state interface", joint.name.c_str());
            return CallbackReturn::ERROR;
        }

        joint_names_.push_back(joint.name);
        zero_positions_.push_back(JOINT_ZERO_POSITIONS.at(joint.name));
        position_states_.push_back(0.0);
        position_commands_.push_back(0.0);
    }

    arduino_to_internal_.resize(6, -1);
    internal_to_arduino_.resize(joint_names_.size(), -1);

    for (int arduino_idx = 0; arduino_idx < 6; arduino_idx++) {
        const std::string & name = ARDUINO_ORDER[arduino_idx];
        auto it = std::find(joint_names_.begin(), joint_names_.end(), name);
        if (it == joint_names_.end()) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "Arduino joint '%s' not found in URDF joints", name.c_str());
            return CallbackReturn::ERROR;
        }
        int internal_idx = std::distance(joint_names_.begin(), it);
        arduino_to_internal_[arduino_idx] = internal_idx;
        internal_to_arduino_[internal_idx] = arduino_idx;
    }

    return CallbackReturn::SUCCESS;
}

// export_state_interfaces 

std::vector<hardware_interface::StateInterface>
ArmHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < joint_names_.size(); i++) {
        state_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &position_states_[i]);
    }
    return state_interfaces;
}

// export_command_interfaces 

std::vector<hardware_interface::CommandInterface>
ArmHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < joint_names_.size(); i++) {
        command_interfaces.emplace_back(
            joint_names_[i],
            hardware_interface::HW_IF_POSITION,
            &position_commands_[i]);
    }
    return command_interfaces;
}

// on_activate 

CallbackReturn ArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (use_serial_) {
        // -- Open serial port --
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ == -1) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "Failed to open serial port %s: %s",
                serial_port_.c_str(), strerror(errno));
            return CallbackReturn::ERROR;
        }

        // -- Configure termios --
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "tcgetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return CallbackReturn::ERROR;
        }
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |=  CS8;
        tty.c_cflag |=  CREAD | CLOCAL;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);
        tty.c_oflag &= ~OPOST;
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_FATAL(rclcpp::get_logger("ArmHardwareInterface"),
                "tcsetattr failed: %s", strerror(errno));
            close(serial_fd_);
            serial_fd_ = -1;
            return CallbackReturn::ERROR;
        }

        // FIX 2: Wait for Arduino to finish booting after serial open
        // (opening the port triggers a DTR reset on most Arduinos)
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Cycling DTR to reset usbipd RX pipe...");

        int flags;
        ioctl(serial_fd_, TIOCMGET, &flags);
        flags &= ~TIOCM_DTR;                                            // DTR low
        ioctl(serial_fd_, TIOCMSET, &flags);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        flags |= TIOCM_DTR;                                             // DTR high
        ioctl(serial_fd_, TIOCMSET, &flags);

        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Waiting 2s for Arduino to boot...");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        tcflush(serial_fd_, TCIOFLUSH);
    }

    // -- Initialize safety state --
    // REPLACE with:
    last_write_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    for (int i = 0; i < 6; i++)
        last_arduino_values_[i] = static_cast<int>(zero_positions_[arduino_to_internal_[i]]);

    // -- Send home --
    send_home();

    RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
        "Activated — serial port %s open at B9600", serial_port_.c_str());

    return CallbackReturn::SUCCESS;
}

// on_deactivate 

CallbackReturn ArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (serial_fd_ != -1) {
        send_home();
        tcdrain(serial_fd_);
        close(serial_fd_);
        serial_fd_ = -1;
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Deactivated — serial port closed");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Deactivated — dry run mode (no serial)");
    }
    return CallbackReturn::SUCCESS;
}

//  send_home

void ArmHardwareInterface::send_home()
{
    if (serial_fd_ == -1) return;
    std::ostringstream cmd;
    cmd << "c1,d";
    for (const auto & name : ARDUINO_ORDER)
        cmd << "," << static_cast<int>(JOINT_ZERO_POSITIONS.at(name));
    cmd << "," << FINGERS_ZERO;
    cmd << "," << THUMB_ZERO;
    cmd << "\r";
    std::string s = cmd.str();
    ssize_t written = ::write(serial_fd_, s.c_str(), s.size());
    if (written < 0)
        RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
            "send_home write failed: %s", strerror(errno));
    else
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Sent home: %s", s.c_str());
}

// write 

hardware_interface::return_type ArmHardwareInterface::write(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/)
{
    if (serial_fd_ == -1)
        return hardware_interface::return_type::OK;  // dry run / no serial

    // -- Enforce minimum delay between commands (100ms) --
    if ((time - last_write_time_).seconds() < 0.1)
        return hardware_interface::return_type::OK;

    // -- Convert position_commands_ (radians) to Arduino values --
    std::array<int, 8> arduino_values;
    for (int i = 0; i < 6; i++) {
        int internal_idx = arduino_to_internal_[i];
        double rad  = position_commands_[internal_idx];
        double zero = zero_positions_[internal_idx];
        arduino_values[i] = static_cast<int>(std::round(zero + (rad * 180.0 / M_PI)));

        // FIX 1: Clamp to valid servo range (0–180), not 0–255
        arduino_values[i] = std::clamp(arduino_values[i], 0, 180);
    }
    arduino_values[6] = FINGERS_ZERO;
    arduino_values[7] = THUMB_ZERO;

    // -- Clamp max step per joint (15 deg backstop) --
    for (int i = 0; i < 6; i++) {
        int step = std::abs(arduino_values[i] - last_arduino_values_[i]);
        if (step > static_cast<int>(MAX_STEP)) {
            int direction = (arduino_values[i] > last_arduino_values_[i]) ? 1 : -1;
            arduino_values[i] = last_arduino_values_[i] + direction * static_cast<int>(MAX_STEP);
            RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                "Arduino slot %d step clamped to %.0f deg", i, MAX_STEP);
        }
        last_arduino_values_[i] = arduino_values[i];
    }

    std::ostringstream cmd;    // ← add this line
    // Both send_home() and write():
    cmd << "c1,d";
    for (int i = 0; i < 8; i++)
        cmd << "," << arduino_values[i];
    cmd << "\r";

    // -- Send --
    std::string s = cmd.str();
    ssize_t written = ::write(serial_fd_, s.c_str(), s.size());
    if (written < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ArmHardwareInterface"),
            "Serial write failed: %s", strerror(errno));
        return hardware_interface::return_type::ERROR;
    }

    last_write_time_ = time;
    return hardware_interface::return_type::OK;
}

// read

hardware_interface::return_type ArmHardwareInterface::read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/)
{
    if (serial_fd_ == -1)
        return hardware_interface::return_type::OK;

    if (read_buffer_.size() > 1024) {
        RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
            "read_buffer_ overflow (%zu bytes), clearing", read_buffer_.size());
        read_buffer_.clear();
    }

    // -- Read available bytes into buffer --
    char tmp[256];
    ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));
    if (n > 0) {
        read_buffer_.append(tmp, n);
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Read %zd bytes, buffer: '%s'", n, read_buffer_.c_str());
    }

    // -- Check for a complete line --
    size_t newline_pos = read_buffer_.find('\n');
    if (newline_pos == std::string::npos)
        return hardware_interface::return_type::OK;

    // -- Extract line, keep remainder --
    std::string line = read_buffer_.substr(0, newline_pos);
    read_buffer_ = read_buffer_.substr(newline_pos + 1);

    // -- Trim carriage return if present --
    if (!line.empty() && line.back() == '\r')
        line.pop_back();

    // -- Validate frame header --
    if (line.empty() || line[0] != 'f') {
        RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
            "Unexpected frame: '%s'", line.c_str());
        return hardware_interface::return_type::OK;
    }

    // -- Parse 6 values --
    std::istringstream ss(line.substr(1));
    std::string token;
    std::vector<int> raw_values;
    raw_values.reserve(6);
    while (std::getline(ss, token, ',')) {
        try {
            raw_values.push_back(std::stoi(token));
        } catch (const std::exception & e) {
            RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
                "Failed to parse token '%s': %s", token.c_str(), e.what());
            return hardware_interface::return_type::OK;
        }
    }

    if (raw_values.size() != 6) {
        RCLCPP_WARN(rclcpp::get_logger("ArmHardwareInterface"),
            "Expected 6 values, got %zu", raw_values.size());
        return hardware_interface::return_type::OK;
    }

    // -- Convert raw ADC (0-255) to radians and store --
    for (int arduino_idx = 0; arduino_idx < 6; arduino_idx++) {
        int internal_idx = arduino_to_internal_[arduino_idx];
        double zero = zero_positions_[internal_idx];
        double deg = (raw_values[arduino_idx] / 255.0) * 180.0;  // scale to degrees
        double rad = (raw_values[arduino_idx] - zero) / 255.0 * joint_range_rad_[internal_idx];
        position_states_[internal_idx] = rad;
        position_states_[internal_idx] = rad;
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
            "Joint %s: raw=%d  zero=%.0f  → %.4f rad (%.1f deg)",
            joint_names_[internal_idx].c_str(),   // fixed: internal_idx not i
            raw_values[arduino_idx],
            zero,
            rad,
            rad * 180.0 / M_PI);
    }

    return hardware_interface::return_type::OK;
   
}

}  // namespace my_robot_controller

PLUGINLIB_EXPORT_CLASS(
    my_robot_controller::ArmHardwareInterface,
    hardware_interface::SystemInterface)