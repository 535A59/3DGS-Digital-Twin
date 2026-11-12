#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_ros/conversions.hpp>          // 点云→OctoMap
#include <octomap_msgs/conversions.h>          

class CloudToOctomap : public rclcpp::Node
{
public:
  CloudToOctomap() :
    Node("cloud_to_octomap"),
    tree_(declare_parameter("resolution", 0.02))
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    pub_ = create_publisher<octomap_msgs::msg::Octomap>(
             "/octomap_binary",
             rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
             "/background_cloud_reliable",
             rclcpp::SensorDataQoS(),
             std::bind(&CloudToOctomap::cloudCallback, this, std::placeholders::_1));
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    /* 1. PointCloud2 -> octomap::Pointcloud */
    octomap::Pointcloud pc;
    octomap::pointCloud2ToOctomap(*msg, pc);

    /* 2. 插入八叉树：激光原点可根据实际 tf 改写 */
    tree_.insertPointCloud(pc, octomap::point3d(0, 0, 0));

    /* 3. 树 -> Octomap 消息并发布 */
    octomap_msgs::msg::Octomap m;
    m.header.stamp    = msg->header.stamp;
    m.header.frame_id = frame_id_;
    octomap_msgs::binaryMapToMsg(tree_, m);
    pub_->publish(m);
  }

  octomap::OcTree tree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr    pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  std::string frame_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToOctomap>());
  rclcpp::shutdown();
  return 0;
}