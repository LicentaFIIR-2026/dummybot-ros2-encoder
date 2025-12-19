#ifndef DUMMYBOT_HARDWARE_INTERFACE_HPP_
#define DUMMYBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dummybot_hardware
{

struct JointConfig {
    std::string name;           // Joint name (ex: "front_left_wheel_joint")
    int encoder_index;          // 0=FL, 1=FR, 2=RL, 3=RR
    double calibration_factor;  // Factor de calibrare (din EEPROM ESP32)
};

class DummyBotHardwareInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(DummyBotHardwareInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;

    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Parameters from URDF
    std::string serial_port_;
    int serial_baud_;
    double wheel_radius_;
    double wheel_separation_;
    int ticks_per_revolution_;
    
    // Calibration factors (loaded from ESP32 or URDF)
    std::vector<double> calibration_factors_;  // FL, FR, RL, RR
    
    // Serial communication
    int serial_fd_;
    
    // Joint configuration
    std::vector<JointConfig> joint_configs_;
    
    // Hardware states (position, velocity, effort)
    std::vector<double> hw_commands_;      // Commanded velocities (rad/s)
    std::vector<double> hw_positions_;     // Joint positions (rad)
    std::vector<double> hw_velocities_;    // Joint velocities (rad/s)
    std::vector<double> hw_efforts_;       // Joint efforts (N⋅m) - not used
    
    // Encoder tracking
    std::vector<int32_t> last_encoder_counts_;
    rclcpp::Time last_read_time_;
    bool first_read_;
    
    // Helper methods - Serial communication
    int init_serial(const std::string& port_name, int baud_rate);
    void close_serial();
    bool send_command(const std::string& cmd);
    std::string read_response(int timeout_ms = 100);
    
    // Helper methods - ESP32 protocol
    bool reset_encoders();
    std::vector<int32_t> read_encoders();
    bool send_motor_speeds(int32_t fl, int32_t fr, int32_t rl, int32_t rr);
    bool load_calibration_from_esp32();
    
    // Helper methods - Conversions
    double encoder_ticks_to_radians(int32_t ticks);
    int32_t radians_per_sec_to_ticks_per_frame(double rad_per_sec);
};

}  // namespace dummybot_hardware

#endif  // DUMMYBOT_HARDWARE_INTERFACE_HPP_