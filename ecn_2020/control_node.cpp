// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

using geometry_msgs::msg::Twist;
using sensor_msgs::msg::JointState;


namespace ecn_2020
{

class ControlNode : public rclcpp::Node
{
public:
  ControlNode(rclcpp::NodeOptions options)
    : Node("control", options), tf_buffer(get_clock()), tf_listener(tf_buffer)
  {
    // init parameters: name of this robot + name of the target
    my_frame = declare_parameter("frame", "base_link");
    target = declare_parameter("target", "target");

    const auto cmd_topic{"cmd_vel"};
    const auto joint_topic{"joint_states"};

    cmd_pub = create_publisher<Twist>(cmd_topic, 10);
    js_pub = create_publisher<JointState>(joint_topic, 10);

    state.name = {"wheel", "torso", "neck"};
    state.position = {0,0,0};


    // init whatever is needed for your node: publisher / timer / etc
    cmd_timer = create_wall_timer(100ms, [this](){update();});

  }
  
private:

  // declare member variables for publisher and timer
  rclcpp::Publisher<Twist>::SharedPtr cmd_pub;
  rclcpp::Publisher<JointState>::SharedPtr js_pub;
  JointState state;

  rclcpp::TimerBase::SharedPtr cmd_timer;

  // TF 2 stuff
  std::string my_frame, target;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  void update()
  {
    //const auto timeout{tf2::durationFromSec(0.05)};

    if(!tf_buffer.canTransform(target, my_frame, tf2::TimePointZero))
      return;

    const auto pose{tf_buffer.lookupTransform(my_frame, target, tf2::TimePointZero).transform};
    Twist cmd;
    cmd.linear.x = 2*(pose.translation.x-1);
    cmd.angular.z = 2*atan2(pose.translation.y, pose.translation.x);
    cmd_pub->publish(cmd);


    state.header.stamp = get_clock()->now();

    state.position[0] += 3.7*cmd.linear.x * 0.1;
    state.position[1] = cmd.linear.x * M_PI/12;
    state.position[2] = cmd.angular.z * M_PI/8;
    js_pub->publish(state);
  }

  
};
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ecn_2020::ControlNode)
