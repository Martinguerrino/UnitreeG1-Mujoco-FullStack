#ifndef G1_RVIZ_PLUGINS__PERCEPTION_PANEL_HPP_
#define G1_RVIZ_PLUGINS__PERCEPTION_PANEL_HPP_
#include <mutex>
#include <string>
#include <QLabel>
#include <QTableWidget>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/string.hpp>

namespace g1_rviz_plugins
{
class PerceptionPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit PerceptionPanel(QWidget * parent = nullptr);
  void onInitialize() override;
protected:
  void bindNode(const rclcpp::Node::SharedPtr & node);
  void receive(const std::string & payload);
  void refresh();
private:
  QLabel * status_;
  QTableWidget * table_;
  QTimer * timer_;
  std::mutex mutex_;
  std::string payload_;
  std::chrono::steady_clock::time_point received_;
  bool has_data_ = false;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
}
#endif
