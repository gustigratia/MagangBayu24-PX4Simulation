#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_control_mode.hpp"
#include <chrono>
#include <iostream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

float rad(int degree){
  float result = M_PI * degree / 180; 
  return result;
}

class OffboardControl : public rclcpp::Node
{
  public:
    OffboardControl() : Node("star_shape")
    {
      offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/in/vehicle_command", 10);

      offboard_setpoint_counter = 0;
      auto timer_callback = [this]() -> void {
        if(offboard_setpoint_counter==10){
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }
        
        if(offboard_setpoint_counter < 100){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(0.0,0.0,-5.0,0.0);
        }
        else if(offboard_setpoint_counter >= 100 && offboard_setpoint_counter < 150){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(0.0,0.0,-5.0,-rad(18));
        }
        else if(offboard_setpoint_counter >= 150 && offboard_setpoint_counter < 220){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(5.4,-1.8,-5.0,-rad(18));
        }
        else if(offboard_setpoint_counter >= 220 && offboard_setpoint_counter < 250){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(5.4,-1.8,-5.0,rad(144));
        }
        else if(offboard_setpoint_counter >= 250 && offboard_setpoint_counter < 320){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(2.0,2.8,-5.0,rad(144));
        }
        else if(offboard_setpoint_counter >= 320 && offboard_setpoint_counter < 350){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(2.0,2.8,-5.0,-rad(90));
        }
        else if(offboard_setpoint_counter >= 350 && offboard_setpoint_counter < 420){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(2.0,-2.8,-5.0,-rad(90));
        }
        else if(offboard_setpoint_counter >= 420 && offboard_setpoint_counter < 450){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(2.0,-2.8,-5.0,rad(36));
        }
        else if(offboard_setpoint_counter >= 450 && offboard_setpoint_counter < 520){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(5.4,1.8,-5.0,rad(36));
        }
        else if(offboard_setpoint_counter >= 520 && offboard_setpoint_counter < 550){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(5.4,1.8,-5.0,-rad(144));
        }
        else if(offboard_setpoint_counter >= 550 && offboard_setpoint_counter < 620){
          publish_offboard_control_mode();
          publish_trajectory_setpoint(0.0,0.0,-5.0,-rad(144));
        }
        else if (offboard_setpoint_counter == 630){
          this->land();
        }
        else if (offboard_setpoint_counter == 750){
          this->disarm();
        }
        if(offboard_setpoint_counter < 751){
          offboard_setpoint_counter++;
        }
      };

      timer_ = this->create_wall_timer(100ms, timer_callback);
    }

    void arm();
    void disarm();
    void land();

  private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;//t
    uint64_t offboard_setpoint_counter;

    void publish_offboard_control_mode();
	  void publish_trajectory_setpoint(float x, float y, float z, float yaw);
	  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
};

void OffboardControl::arm(){
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
  RCLCPP_INFO(this->get_logger(), "Arm command send...");
}

void OffboardControl::disarm(){
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
	RCLCPP_INFO(this->get_logger(), "Disarm command send...");
}

void OffboardControl::land(){
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_VTOL_LAND);
  RCLCPP_INFO(this->get_logger(), "Land command send...");
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