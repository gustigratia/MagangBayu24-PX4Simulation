#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class OffboardControl : public rclcpp::Node {
    public:
        OffboardControl() : Node("circle_shape") {
            trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
            offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
            vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
            offboard_setpoint_counter_ = 0;
            yawcount = 1;
            auto timer_callback = [this]() -> void {
                if (offboard_setpoint_counter_ == 100) {
				    this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				    this->arm();
			    }
                if (offboard_setpoint_counter_ < 190){
                    publish_offboard_control_mode();
                    takeoff();
                }
                else if(offboard_setpoint_counter_ >= 190 && offboard_setpoint_counter_ < 440){
                    publish_offboard_control_mode();
                    publish_trajectory_setpoint(0.1-yawcount*0.0255);
                    yawcount++;
                }
                else if (offboard_setpoint_counter_ == 500){
                    this->land();
                }
                if (offboard_setpoint_counter_ == 650) {
                    this->disarm();
                }
                if (offboard_setpoint_counter_ < 651){
				    offboard_setpoint_counter_++;
			    }
                
            };
            timer_ = this->create_wall_timer(100ms, timer_callback);
        }

        void arm();
        void land();
        void takeoff();
	    void disarm();

    private:
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;//t
        rclcpp::TimerBase::SharedPtr timer_;
        std::atomic<uint64_t> timestamp_;
        uint64_t offboard_setpoint_counter_;
        uint64_t yawcount; 
        void publish_offboard_control_mode();
	    void publish_trajectory_setpoint(float yaw);
	    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
        
};

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void OffboardControl::land(){
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_VTOL_LAND, 0.0);
  RCLCPP_INFO(this->get_logger(), "Land command send");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint(float yaw)
{
	TrajectorySetpoint msg{};
    float radius = 3.0;
    float omega = 0.2514;
    double time_now = static_cast<double>(this->get_clock()->now().nanoseconds()) / 1e9; 
    float x = radius * cos(omega*time_now)-3;
    float y = radius * sin(omega*time_now);
	msg.position = {-x, y,-3.0};
	msg.yaw = yaw;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::takeoff()
{
    TrajectorySetpoint msg{};
    msg.position = {0.0,0.0,-5.0};
    msg.yaw = 0.0;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}


int main(int argc, char *argv[]) {
  std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}