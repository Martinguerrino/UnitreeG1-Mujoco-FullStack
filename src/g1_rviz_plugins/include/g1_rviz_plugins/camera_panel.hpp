#ifndef G1_RVIZ_PLUGINS__CAMERA_PANEL_HPP_
#define G1_RVIZ_PLUGINS__CAMERA_PANEL_HPP_

#include <atomic>
#include <QLabel>
#include <QSlider>
#include <QSpinBox>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/float64.hpp>

namespace g1_rviz_plugins
{
class CameraPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit CameraPanel(QWidget * parent = nullptr);
  void onInitialize() override;
protected:
  void bindNode(const rclcpp::Node::SharedPtr & node);
private:
  void command(int degrees);
  QSlider * slider_;
  QSpinBox * angle_;
  QLabel * status_;
  QTimer * timer_;
  bool selected_ = false;
  bool initialized_angle_ = false;
  std::atomic<double> observed_{0.0};
  std::atomic<bool> received_{false};
  std::atomic<int64_t> last_received_ns_{0};
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};
}  // namespace g1_rviz_plugins
#endif
