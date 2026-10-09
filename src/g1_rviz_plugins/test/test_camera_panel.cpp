#include <gtest/gtest.h>
#include <QApplication>
#include <QPushButton>
#include <QSlider>
#include <pluginlib/class_loader.hpp>
#include "g1_rviz_plugins/camera_panel.hpp"

namespace
{
class TestPanel : public g1_rviz_plugins::CameraPanel
{
public:
  using CameraPanel::bindNode;
};

TEST(CameraPanel, SliderAndResetPublishCommands)
{
  int argc = 1;
  char name[] = "test_camera_panel";
  char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("camera_panel_test");
  std::vector<double> received;
  auto subscription = node->create_subscription<std_msgs::msg::Float64>(
    "/camera/pitch_command", 10, [&received](std_msgs::msg::Float64::ConstSharedPtr msg) {
      received.push_back(msg->data);
    });
  {
    // Exercise discovery and construction through RViz's actual plugin registry.
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto plugin = loader.createSharedInstance("g1_rviz_plugins/CameraPanel");
    EXPECT_NE(plugin->findChild<QSlider *>("camera_pitch_slider"), nullptr);
    TestPanel panel;
    panel.bindNode(node);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (node->count_subscribers("/camera/pitch_command") == 0 &&
      std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto * slider = panel.findChild<QSlider *>("camera_pitch_slider");
    auto * reset = panel.findChild<QPushButton *>("camera_pitch_reset");
    ASSERT_NE(slider, nullptr);
    ASSERT_NE(reset, nullptr);
    EXPECT_EQ(slider->minimum(), -30);
    EXPECT_EQ(slider->maximum(), 45);
    slider->setValue(25);
    reset->click();
    reset->click();  // Reset also publishes when already at zero.
    while (received.size() < 3 && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(received.size(), 3u);
    EXPECT_EQ(received[0], 25);
    EXPECT_EQ(received[1], 0);
    EXPECT_EQ(received[2], 0);
  }
  subscription.reset();
  node.reset();
  rclcpp::shutdown();
}
}  // namespace
