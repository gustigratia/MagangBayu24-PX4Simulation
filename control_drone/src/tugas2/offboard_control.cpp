#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class OffboardControl : public rclcpp::Node {

public:
  OffboardControl() : Node("rectangle_shape") {
    trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
    offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}

      if(offboard_setpoint_counter_ < 100){
        publish_offboard_control_mode();
        publish_trajectory_setpoint(0.0,0.0,-5.0,0.0);
      }
      else if (offboard_setpoint_counter_>= 100 && offboard_setpoint_counter_ < 150){ 
        publish_offboard_control_mode();
        publish_trajectory_setpoint(3.0,0.0,-5.0,0.0);
      }
      else if(offboard_setpoint_counter_ >= 150 && offboard_setpoint_counter_ < 180){ // yaw
        publish_offboard_control_mode();
        publish_trajectory_setpoint(3.0,0.0,-5.0,1.57);
      }
      else if (offboard_setpoint_counter_ > 180 && offboard_setpoint_counter_ < 230){
        publish_offboard_control_mode();
        publish_trajectory_setpoint(3.0,6.0,-5.0,1.57);
      }
      else if(offboard_setpoint_counter_ >= 230 && offboard_setpoint_counter_ < 260){ // yaw
        publish_offboard_control_mode();
        publish_trajectory_setpoint(3.0,6.0,-5.0,-3.14);
      }
      else if (offboard_setpoint_counter_ > 260 && offboard_setpoint_counter_ < 310){
        publish_offboard_control_mode();
        publish_trajectory_setpoint(-3.0,6.0,-5.0,-3.14);
      }
      else if(offboard_setpoint_counter_ >= 310 && offboard_setpoint_counter_ < 340){ // yaw
        publish_offboard_control_mode();
        publish_trajectory_setpoint(-3.0,6.0,-5.0,-1.57);
      }
      else if (offboard_setpoint_counter_ > 340 && offboard_setpoint_counter_ < 390){ 
        publish_offboard_control_mode();
        publish_trajectory_setpoint(-3.0,0.0,-5.0,-1.57);
      }
      else if(offboard_setpoint_counter_ >= 390 && offboard_setpoint_counter_ < 420){ // yaw
        publish_offboard_control_mode();
        publish_trajectory_setpoint(-3.0,0.0,-5.0,0.0);
      }
      else if (offboard_setpoint_counter_ > 420 && offboard_setpoint_counter_ < 470){
        publish_offboard_control_mode();
        publish_trajectory_setpoint(0.0,0.0,-5.0,0.0);
      }
      else if (offboard_setpoint_counter_ == 490){
        this->land();
      }
      if (offboard_setpoint_counter_ == 690){
        this->disarm();
      }
			if (offboard_setpoint_counter_ < 691){
				offboard_setpoint_counter_++;
			}
		};
    timer_ = this->create_wall_timer(100ms, timer_callback);
  }
  void arm();
  void land();
	void disarm();

private:
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;//t
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> timestamp_;
  uint64_t offboard_setpoint_counter_; 
  void publish_offboard_control_mode();
	void publish_trajectory_setpoint(float x, float y, float z, float yaw);
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

void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x,y,z};
	msg.yaw = yaw;
  msg.velocity = {};
  msg.acceleration = {};
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