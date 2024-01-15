// mandatory includes for base code
#include <rclcpp/rclcpp.hpp>
#include <ecn_2022/eigen.h>

// include any thing required - do not forget to use the .hpp extension for ROS 2 files





using namespace std::chrono_literals;

const std::vector<std::string> suffixes = {"_s0", "_s1", "_e0", "_e1", "_w0", "_w1", "_w2"};

class ControlNode : public rclcpp::Node
{
public:
  ControlNode() : Node("control")
  {
    // init member variables such as publishers / subscribers / timers




  }
  
private:
  // declare member variables such as publishers / subscribers / timers



  // declare callbacks that are too large to be a simple lambda function



};



// boilerplate main function

int main(int argc, char** argv)
{   
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
