#pragma once

#include <chrono>
#include <memory>
#include <QString>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/float64.hpp>

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTimer;

namespace g1_rviz_plugins
{
class CameraAnglePanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit CameraAnglePanel(QWidget * parent = nullptr);
  void onInitialize() override;
  // Also permits integration tests without starting an entire RViz window.
  void connectRosNode(const rclcpp::Node::SharedPtr & node);

Q_SIGNALS:
  void targetReceived(double value);
  void capturedReceived(double value);
  void originalReceived(double value);
  void requestFinished(bool success, QString reason);

private Q_SLOTS:
  void updateTarget(double value);
  void updateCaptured(double value);
  void updateOriginal(double value);
  void requestAngle(double value);
  void refreshConnection();

private:
  QDoubleSpinBox * editor_;
  QLabel * captured_;
  QLabel * status_;
  QPushButton * reset_;
  QTimer * timer_;
  rclcpp::AsyncParametersClient::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_, captured_sub_;
  bool have_target_ = false, have_original_ = false, original_pending_ = false;
  bool editing_ = false;
  double original_ = 0.;
  std::chrono::steady_clock::time_point last_target_, last_original_request_;
};
}  // namespace g1_rviz_plugins
