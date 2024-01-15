// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include "urdf/target.h"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ecn_2021
{

class ControlNode : public rclcpp::Node
{
  // helper class to see the tracked point in RViz
  TargetPublisher target_pub;

  // TODO declare member variables for publishers, subscribers, timers and others if needed
  std::string robot_name{"turtlebot2"};


public:
  ControlNode(rclcpp::NodeOptions options)
    : Node("control", options)
  {
    // TODO init parameters: distance to track the target and robot name


    // TODO use target_pub to see the tracked point in RViz
    target_pub.init(this, robot_name);


    // TODO init whatever is needed for your node: publishers / subscribers / timers / member variables

  }
  
private:

  void move()
  {
    // find nearest target: finish the findClosest function
    const auto target{findClosest()};
    if(!target.isValid()) return;

    // debug
    target_pub.publish(target);

    // TODO compute control law


    // TODO publish control law



  }


  Target findClosest() const
  {
    // a Target is a range [m] and an angle [rad], default to an invalid one
    Target closest;

    // TODO closest should correspond to the smallest measured range
    // as long as it is between range_min and range_max




    return closest;
  }


};
}


// boilerplate main function, nothing to do here
int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ecn_2021::ControlNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
