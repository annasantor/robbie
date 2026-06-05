#ifndef MYROBOT_INTERFACE_H
#define MYROBOT_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <vector>
#include <string>
#include <unordered_map>
#include <termios.h>
#include <array>
#include <thread>    
#include <chrono>  

namespace my_robot_controller
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmHardwareInterface)

    ArmHardwareInterface();
    ~ArmHardwareInterface();

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface>  export_state_interfaces()  override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read (const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // State and command storage (info_.joints order, driven by URDF)
    std::vector<double> position_states_;
    std::vector<double> position_commands_;

    bool use_serial_ = false;

    // Joint metadata (populated at on_init from info_.joints)
    std::vector<std::string> joint_names_;
    std::vector<double> zero_positions_;        // Arduino home units, matched to joint_names_ order
    std::vector<double> joint_range_rad_;


    // Arduino <-> internal index mappings (computed once at on_init)
    std::vector<int> arduino_to_internal_;      // arduino slot i  -> internal index
    std::vector<int> internal_to_arduino_;      // internal index i -> arduino slot

    rclcpp::Time last_write_time_;
    std::array<int, 6> last_arduino_values_;

    // FIX 1: MAX_STEP is 15 servo degrees — also used as the safe clamp ceiling
    static constexpr double MAX_STEP = 15.0;

    // Serial
    std::string serial_port_;
    int baud_rate_;
    int serial_fd_ = -1;

    // Partial-read accumulator (persists across read() calls)
    std::string read_buffer_;

    // Static name -> Arduino zero position lookup (defined in .cpp)
    static const std::vector<std::string> ARDUINO_ORDER;
    static const std::unordered_map<std::string, double> JOINT_ZERO_POSITIONS;

    static constexpr int FINGERS_ZERO = 160;
    static constexpr int THUMB_ZERO   = 30;

    void send_home();
};

}  // namespace my_robot_controller

#endif  // MYROBOT_INTERFACE_H