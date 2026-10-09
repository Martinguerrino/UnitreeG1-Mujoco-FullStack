#include <gtest/gtest.h>
#include <thread>
#include <QApplication>
#include <QLabel>
#include <QTableWidget>
#include <pluginlib/class_loader.hpp>
#include "g1_rviz_plugins/perception_panel.hpp"

namespace
{
class TestPerceptionPanel : public g1_rviz_plugins::PerceptionPanel
{
public:
  using PerceptionPanel::receive;
  using PerceptionPanel::refresh;
};

TEST(PerceptionPanel, ObjectsReasonsAndInvalidDataClearRows)
{
  int argc = 1;
  char name[] = "test_perception_panel";
  char * argv[] = {name, nullptr};
  QApplication app(argc, argv);
  pluginlib::ClassLoader<rviz_common::Panel> loader("rviz_common", "rviz_common::Panel");
  auto plugin = loader.createSharedInstance("g1_rviz_plugins/PerceptionPanel");
  EXPECT_NE(plugin->findChild<QTableWidget *>("perception_objects"), nullptr);
  TestPerceptionPanel panel;
  auto * table = panel.findChild<QTableWidget *>("perception_objects");
  ASSERT_NE(table, nullptr);
  panel.receive(R"({"status":"ok","frame_id":"pelvis","tables":[
    {"id":"table_1","confirmed":true,"stable":true,"age_sec":0,
     "dimensions":[1,0.7,0],"objects":[
       {"id":"box_2","confirmed":true,"stable":false,"age_sec":0,
        "dimensions":[0.1,0.1,0.1],"stability":{"reasons":["position_variation"]}}
     ]}]})");
  panel.refresh();
  ASSERT_EQ(table->rowCount(), 2);
  EXPECT_EQ(table->item(0, 2)->text(), "Estable");
  EXPECT_EQ(table->item(1, 0)->text(), "box_2");
  EXPECT_EQ(table->item(1, 2)->text(), "Inestable");
  EXPECT_EQ(table->item(1, 6)->text(), "Posición variable");
  panel.receive(R"({"status":"ok","tables":[{"id":"box_2","stable":true,"age_sec":1}]})");
  panel.refresh();
  EXPECT_EQ(table->item(0, 2)->text(), "Antigua");
  panel.receive(R"({"status":"no_table","tables":[]})");
  panel.refresh();
  EXPECT_EQ(table->rowCount(), 0);
  panel.receive(R"({"status":"ok","tables":[{"id":"box_2","stable":true,"age_sec":0}]})");
  panel.refresh();
  EXPECT_EQ(table->rowCount(), 1);
  std::this_thread::sleep_for(std::chrono::milliseconds(2100));
  panel.refresh();
  EXPECT_EQ(table->rowCount(), 0);
  panel.receive("broken json");
  panel.refresh();
  EXPECT_EQ(table->rowCount(), 0);
}
}
