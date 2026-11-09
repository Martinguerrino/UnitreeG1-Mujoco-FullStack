#include "g1_rviz_plugins/camera_angle_panel.hpp"
#include <cmath>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPointer>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace g1_rviz_plugins
{
CameraAnglePanel::CameraAnglePanel(QWidget * parent) : rviz_common::Panel(parent)
{
  editor_ = new QDoubleSpinBox(this);
  editor_->setObjectName("camera_pitch_editor");
  editor_->setRange(-45., 90.);
  editor_->setDecimals(3);
  editor_->setSingleStep(.1);
  editor_->setSuffix(QString::fromUtf8(" °"));
  editor_->setKeyboardTracking(false);  // Commit typed numbers with Enter/focus loss.
  editor_->setEnabled(false);
  editor_->setToolTip(QString::fromUtf8("Inclinación respecto del torso. Positivo: hacia abajo. Enter para aplicar."));
  captured_ = new QLabel(QString::fromUtf8("—"), this);
  captured_->setObjectName("camera_pitch_captured");
  reset_ = new QPushButton("Restaurar URDF", this);
  reset_->setObjectName("camera_pitch_reset");
  reset_->setEnabled(false);
  status_ = new QLabel("Esperando control de cámara…", this);
  status_->setWordWrap(true);
  auto form = new QFormLayout();
  form->addRow(QString::fromUtf8("Ángulo (°):"), editor_);
  form->addRow(QString::fromUtf8("Última imagen:"), captured_);
  auto layout = new QVBoxLayout(this);
  layout->addLayout(form);
  layout->addWidget(reset_);
  layout->addWidget(status_);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &CameraAnglePanel::refreshConnection);
  connect(editor_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &CameraAnglePanel::requestAngle);
  connect(editor_->findChild<QLineEdit *>(), &QLineEdit::textEdited, this,
    [this]() {editing_ = true;});
  connect(editor_, &QDoubleSpinBox::editingFinished, this, [this]() {editing_ = false;});
  connect(reset_, &QPushButton::clicked, this, [this]() {requestAngle(original_);});
  connect(this, &CameraAnglePanel::targetReceived, this, &CameraAnglePanel::updateTarget, Qt::QueuedConnection);
  connect(this, &CameraAnglePanel::capturedReceived, this, &CameraAnglePanel::updateCaptured, Qt::QueuedConnection);
  connect(this, &CameraAnglePanel::originalReceived, this, &CameraAnglePanel::updateOriginal, Qt::QueuedConnection);
  connect(this, &CameraAnglePanel::requestFinished, this,
    [this](bool success, const QString & reason) {
      status_->setText(success ? "Ángulo aplicado" : "No se aplicó: " + reason);
    }, Qt::QueuedConnection);
}

void CameraAnglePanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (abstraction) {connectRosNode(abstraction->get_raw_node());}
}

void CameraAnglePanel::connectRosNode(const rclcpp::Node::SharedPtr & node)
{
  QPointer<CameraAnglePanel> guard(this);
  client_ = std::make_shared<rclcpp::AsyncParametersClient>(node, "/g1_camera_angle");
  target_sub_ = node->create_subscription<std_msgs::msg::Float64>("/camera/pitch_deg", 10,
    [guard](const std_msgs::msg::Float64::SharedPtr msg) {
      if (guard) {Q_EMIT guard->targetReceived(msg->data);}
    });
  captured_sub_ = node->create_subscription<std_msgs::msg::Float64>("/camera/pitch_captured_deg", 10,
    [guard](const std_msgs::msg::Float64::SharedPtr msg) {
      if (guard) {Q_EMIT guard->capturedReceived(msg->data);}
    });
  timer_->start(500);
}

void CameraAnglePanel::updateTarget(double value)
{
  if (!std::isfinite(value) || value < -45. || value > 90.) {return;}
  last_target_ = std::chrono::steady_clock::now();
  have_target_ = true;
  if (!editing_ || !editor_->isEnabled()) {
    const QSignalBlocker block(editor_);
    editor_->setValue(value);
  }
  refreshConnection();
}

void CameraAnglePanel::updateCaptured(double value)
{
  if (std::isfinite(value)) {captured_->setText(QString::number(value, 'f', 3) + QString::fromUtf8(" °"));}
}

void CameraAnglePanel::updateOriginal(double value)
{
  original_pending_ = false;
  if (!std::isfinite(value) || value < -45. || value > 90.) {return;}
  original_ = value;
  have_original_ = true;
  reset_->setText("Restaurar URDF (" + QString::number(value, 'f', 3) + QString::fromUtf8("°)"));
  refreshConnection();
}

void CameraAnglePanel::refreshConnection()
{
  const auto now = std::chrono::steady_clock::now();
  const bool fresh = have_target_ && (now - last_target_ < std::chrono::seconds(2));
  const bool ready = client_ && client_->service_is_ready();
  editor_->setEnabled(ready && fresh);
  reset_->setEnabled(ready && fresh && have_original_);
  if (!ready || !fresh) {status_->setText("Esperando control de cámara…");}
  if (ready && !have_original_ && (!original_pending_ || now - last_original_request_ > std::chrono::seconds(3))) {
    original_pending_ = true;
    last_original_request_ = now;
    QPointer<CameraAnglePanel> guard(this);
    client_->get_parameters({"urdf_pitch_deg"}, [guard](auto future) {
      if (!guard) {return;}
      const auto values = future.get();
      if (!values.empty() && values.front().get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        Q_EMIT guard->originalReceived(values.front().as_double());
      }
    });
  }
}

void CameraAnglePanel::requestAngle(double value)
{
  if (!client_ || !client_->service_is_ready() || !std::isfinite(value)) {
    status_->setText("Control de cámara no disponible");
    return;
  }
  QPointer<CameraAnglePanel> guard(this);
  client_->set_parameters({rclcpp::Parameter("pitch_deg", value)}, [guard](auto future) {
    if (!guard) {return;}
    const auto result = future.get();
    const bool success = !result.empty() && result.front().successful;
    Q_EMIT guard->requestFinished(success,
      result.empty() ? "Sin respuesta" : QString::fromStdString(result.front().reason));
  });
}
}  // namespace g1_rviz_plugins

PLUGINLIB_EXPORT_CLASS(g1_rviz_plugins::CameraAnglePanel, rviz_common::Panel)
