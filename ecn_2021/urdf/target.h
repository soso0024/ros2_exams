#ifndef EXAM_2021_RANGE
#define EXAM_2021_RANGE

#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

namespace ecn_exam_2021
{

// helper class
struct Target
{
  double range = 100.;
  double angle{std::nan("")};

  explicit inline Target() {}

  inline double computeVx(double distance) const
  {
    const auto pos{range*cos(angle)};
    if(pos < 0) return 0;
    return std::clamp(2.5*(pos-distance), -2.,2.);
  }
  inline double computeOmegaZ() const
  {
    return std::clamp(2.*(fmod(angle+M_PI, 2*M_PI) - M_PI), -2.,2.);
  }
  inline bool isValid() const
  {
    return !std::isnan(angle);
  }
  inline void toPoint(geometry_msgs::msg::Point32& point) const
  {
    point.x = range * cos(angle);
    point.y = range * sin(angle);
  }
};


// helper publisher for RViz
class TargetPublisher
{
  using PointCloud = sensor_msgs::msg::PointCloud;
public:
  inline TargetPublisher() {}
  inline void init(rclcpp::Node* node, std::string robot_name)
  {
    this->node = node;
    target_pub = node->create_publisher<PointCloud>("/" + robot_name + "/target", 10);
    target_msg.header.frame_id = robot_name + "/base_scan";
    target_msg.points.resize(1);
  }

  inline void publish(const Target &target)
  {
    if(!node)
    {
      std::cerr << "You forgot to call init on target_pub" << std::endl;
      return;
    }

    target_msg.header.stamp = node->get_clock()->now();
    target.toPoint(target_msg.points.front());

    target_pub->publish(target_msg);
  }

private:
  rclcpp::Node* node;
  rclcpp::Publisher<PointCloud>::SharedPtr target_pub;
  PointCloud target_msg;
};
}


#endif
