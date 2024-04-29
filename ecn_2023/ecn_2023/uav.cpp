#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <ecn_2023/client_spinner.h>
#include <ecn_2023/srv/target.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using Target = ecn_2023::srv::Target;
using Twist = geometry_msgs::msg::Twist;

constexpr auto dt{0.01};

class UAV : public rclcpp::Node
{
  // helper class to call a service in a synchronous way
  ServiceNodeSync<Target> client{this};

  // PI part
  double Kp{}, Ki{}, Kw{};
  double eix{}, eiy{};

  // TODO declare additional member variables if needed
  // int drone_num{};

public:
  UAV() : Node("uav")
  {
    // control params
    Ki = declare_parameter("Ki", .1);
    Kp = declare_parameter("Kp", 10.);
    Kw = declare_parameter("Kw", 5.);

    // TODO declare a parameter for the drone number
    // drone_num = declare_parameter("drone_num", 1);

    // TODO init the service client
    client.init("/target");

    // TODO init publisher and other things
    publisher = this->create_publisher<Twist>("/uav1/cmd_vel", 10);

    timer = this->create_wall_timer(50ms, std::bind(&UAV::updateControl, this));

  }

  // TODO declare additional member functions if needed
  void updateControl(){
    Target::Request request;
    Target::Response response;

    request.uav = "uav1";

    bool success = client.call(request, response);
    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service target");
        return;
    }

    // Integral error for x and y
    eix += response.x;
    eiy += response.y;

    // Compute desired velocity
    auto message = Twist();
    message.linear.x = Kp * response.x + Ki * eix;
    message.linear.y = Kp * response.y + Ki * eiy;
    message.linear.z = Kp * response.z;
    message.angular.z = Kw * response.theta;

    RCLCPP_INFO(this->get_logger(), "response.x: '%f'", response.x);
    RCLCPP_INFO(this->get_logger(), "response.y: '%f'", response.y);
    RCLCPP_INFO(this->get_logger(), "response.z: '%f'", response.z);

    publisher->publish(message);

    // RCLCPP_INFO(this->get_logger(), "linear.x: '%f'", message.linear.x);
    // RCLCPP_INFO(this->get_logger(), "linear.y: '%f'", message.linear.y);
    // RCLCPP_INFO(this->get_logger(), "linear.z: '%f'", message.linear.z);
  }

  rclcpp::Publisher<Twist>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;

};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UAV>());
  rclcpp::shutdown();
}

/*
[ROS2@baxter] ~/ecn_2023$ ros2 service type /target
ecn_2023/srv/Target

[ROS2@baxter] ~/ecn_2023$ ros2 interface show ecn_2023/srv/Target
string uav
---
float64 x
float64 y
float64 z
float64 theta



[ROS2@baxter] ~/ecn_2023$ ros2 run ecn_2023 uav
[INFO] [1714378189.699504080] [uav]: linear.x: '-3430.612615'
[INFO] [1714378189.699651931] [uav]: linear.y: '74.287178'
[INFO] [1714378189.699687932] [uav]: linear.z: '-55.000000'
[INFO] [1714378189.747783873] [uav]: linear.x: '-3495.328926'
[INFO] [1714378189.747850800] [uav]: linear.y: '35.823051'
[INFO] [1714378189.747875510] [uav]: linear.z: '-55.000000'
[INFO] [1714378189.797767432] [uav]: linear.x: '-3559.793305'
[INFO] [1714378189.797832107] [uav]: linear.y: '-3.436991'
[INFO] [1714378189.797852669] [uav]: linear.z: '-55.000000'
 *
 */
