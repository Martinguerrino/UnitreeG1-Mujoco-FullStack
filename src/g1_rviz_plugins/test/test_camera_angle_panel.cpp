#include <cmath>
#include <memory>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QTest>
#include <QTimer>
#include <pluginlib/class_loader.hpp>
#include "g1_rviz_plugins/camera_angle_panel.hpp"

class CameraAnglePanelTest : public QObject
{
  Q_OBJECT
private Q_SLOTS:
  void discoverPanelPlugin()
  {
    pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
    auto panel = loader.createSharedInstance("g1_rviz_plugins/CameraAnglePanel");
    QVERIFY(panel->findChild<QDoubleSpinBox *>("camera_pitch_editor"));
  }

  void numericEditsEchoDragAndReset()
  {
    int argc = 0;
    rclcpp::init(argc, nullptr);
    auto control = std::make_shared<rclcpp::Node>("g1_camera_angle");
    control->declare_parameter("pitch_deg", 47.6);
    control->declare_parameter("urdf_pitch_deg", 47.6);
    auto target = control->create_publisher<std_msgs::msg::Float64>("/camera/pitch_deg", 10);
    auto captured = control->create_publisher<std_msgs::msg::Float64>("/camera/pitch_captured_deg", 10);
    auto gui_node = std::make_shared<rclcpp::Node>("camera_panel_test");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(control);
    executor.add_node(gui_node);
    QTimer spin;
    connect(&spin, &QTimer::timeout, this, [&]() {
      executor.spin_some();
      std_msgs::msg::Float64 state;
      state.data = control->get_parameter("pitch_deg").as_double();
      target->publish(state);
    });
    spin.start(20);
    {
      g1_rviz_plugins::CameraAnglePanel panel;
      panel.connectRosNode(gui_node);
      panel.show();
      auto editor = panel.findChild<QDoubleSpinBox *>("camera_pitch_editor");
      auto reset = panel.findChild<QPushButton *>("camera_pitch_reset");
      auto actual = panel.findChild<QLabel *>("camera_pitch_captured");
      QVERIFY(editor && reset && actual);
      QTRY_VERIFY_WITH_TIMEOUT(editor->isEnabled() && reset->isEnabled(), 10000);
      QCOMPARE(editor->value(), 47.6);
      QCOMPARE(editor->decimals(), 3);
      editor->setFocus();
      QTest::keyClick(editor, Qt::Key_A, Qt::ControlModifier);
      QTest::keyClicks(editor, "12.345");
      // Incoming state cannot overwrite a number halfway through typing.
      QTest::qWait(100);
      QTest::keyClick(editor, Qt::Key_Return);
      QTRY_VERIFY_WITH_TIMEOUT(std::abs(control->get_parameter("pitch_deg").as_double() - 12.345) < 1e-6, 5000);
      // Actual capture is an independent reading, not an optimistic UI echo.
      std_msgs::msg::Float64 sample;
      sample.data = 11.125;
      captured->publish(sample);
      QTRY_VERIFY_WITH_TIMEOUT(actual->text().startsWith("11.125"), 5000);
      // External parameter change represents dragging the RViz sphere.
      control->set_parameter(rclcpp::Parameter("pitch_deg", 25.125));
      QTRY_COMPARE_WITH_TIMEOUT(editor->value(), 25.125, 5000);
      QTest::mouseClick(reset, Qt::LeftButton);
      QTRY_COMPARE_WITH_TIMEOUT(control->get_parameter("pitch_deg").as_double(), 47.6, 5000);
      QTRY_COMPARE_WITH_TIMEOUT(editor->value(), 47.6, 5000);
    }
    spin.stop();
    executor.remove_node(gui_node);
    executor.remove_node(control);
    rclcpp::shutdown();
  }
};

QTEST_MAIN(CameraAnglePanelTest)
#include "test_camera_angle_panel.moc"
