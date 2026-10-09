#include "g1_rviz_plugins/camera_panel.hpp"

#include <chrono>
#include <cmath>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace g1_rviz_plugins
{
namespace
{
int64_t monotonicNow()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

CameraPanel::CameraPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  layout->addWidget(new QLabel("Inclinación de cámara RGB-D (simulación)", this));
  auto * row = new QHBoxLayout();
  row->addWidget(new QLabel("Arriba", this));
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setObjectName("camera_pitch_slider");
  slider_->setRange(-30, 45);
  slider_->setSingleStep(1);
  slider_->setPageStep(5);
  slider_->setTickPosition(QSlider::TicksBelow);
  slider_->setTickInterval(15);
  row->addWidget(slider_, 1);
  row->addWidget(new QLabel("Abajo", this));
  angle_ = new QSpinBox(this);
  angle_->setObjectName("camera_pitch_angle");
  angle_->setRange(-30, 45);
  angle_->setSuffix("°");
  row->addWidget(angle_);
  layout->addLayout(row);
  auto * reset = new QPushButton("Volver a posición original", this);
  reset->setObjectName("camera_pitch_reset");
  layout->addWidget(reset);
  status_ = new QLabel("Esperando imágenes RGB-D…", this);
  status_->setObjectName("camera_pitch_status");
  layout->addWidget(status_);
  connect(slider_, &QSlider::valueChanged, this, [this](int value) {
    const QSignalBlocker blocker(angle_);
    angle_->setValue(value);
    command(value);
  });
  connect(angle_, qOverload<int>(&QSpinBox::valueChanged), slider_, &QSlider::setValue);
  connect(reset, &QPushButton::clicked, this, [this]() {
    const QSignalBlocker slider_blocker(slider_);
    const QSignalBlocker angle_blocker(angle_);
    slider_->setValue(0);
    angle_->setValue(0);
    command(0);  // Send even if the slider is already at zero.
  });
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() {
    if (!received_ || monotonicNow() - last_received_ns_ > 2000000000LL) {
      status_->setText("Sin imágenes RGB-D recientes");
      return;
    }
    const double applied = observed_;
    status_->setText(QString("Ángulo observado: %1°").arg(applied, 0, 'f', 0));
    if (!initialized_angle_ && !selected_) {
      const QSignalBlocker slider_blocker(slider_);
      const QSignalBlocker angle_blocker(angle_);
      slider_->setValue(static_cast<int>(std::round(applied)));
      angle_->setValue(slider_->value());
      initialized_angle_ = true;
    }
  });
  timer_->start(100);
}

void CameraPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (abstraction) {
    bindNode(abstraction->get_raw_node());
  }
}

void CameraPanel::bindNode(const rclcpp::Node::SharedPtr & node)
{
  publisher_ = node->create_publisher<std_msgs::msg::Float64>("/camera/pitch_command", 10);
  subscription_ = node->create_subscription<std_msgs::msg::Float64>(
    "/camera/pitch_state", 10, [this](std_msgs::msg::Float64::ConstSharedPtr message) {
      if (std::isfinite(message->data)) {
        observed_ = message->data;
        last_received_ns_ = monotonicNow();
        received_ = true;
      }
    });
}

void CameraPanel::command(int degrees)
{
  selected_ = true;
  if (publisher_) {
    std_msgs::msg::Float64 message;
    message.data = degrees;
    publisher_->publish(message);
  }
}
}  // namespace g1_rviz_plugins
PLUGINLIB_EXPORT_CLASS(g1_rviz_plugins::CameraPanel, rviz_common::Panel)
