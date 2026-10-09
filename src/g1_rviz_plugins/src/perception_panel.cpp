#include "g1_rviz_plugins/perception_panel.hpp"
#include <cmath>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace g1_rviz_plugins
{
PerceptionPanel::PerceptionPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);
  status_ = new QLabel("Esperando percepción…", this);
  status_->setObjectName("perception_status");
  layout->addWidget(status_);
  table_ = new QTableWidget(0, 7, this);
  table_->setObjectName("perception_objects");
  table_->setHorizontalHeaderLabels({"Objeto / ID", "Seguimiento", "Estabilidad", "Edad (s)",
    "Dimensiones (m)", "Profundidad", "Motivo"});
  table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->verticalHeader()->hide();
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_->horizontalHeader()->setStretchLastSection(true);
  layout->addWidget(table_);
  auto * note = new QLabel("Estable indica consistencia temporal; no garantiza exactitud.", this);
  note->setWordWrap(true);
  layout->addWidget(note);
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &PerceptionPanel::refresh);
  timer_->start(100);
}

void PerceptionPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (abstraction) {bindNode(abstraction->get_raw_node());}
}

void PerceptionPanel::bindNode(const rclcpp::Node::SharedPtr & node)
{
  subscription_ = node->create_subscription<std_msgs::msg::String>(
    "/perception/detections", 10, [this](std_msgs::msg::String::ConstSharedPtr message) {
      receive(message->data);
    });
}

void PerceptionPanel::receive(const std::string & payload)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  payload_ = payload;
  received_ = std::chrono::steady_clock::now();
  has_data_ = true;
}

void PerceptionPanel::refresh()
{
  std::string payload;
  double elapsed;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_) {return;}
    elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count();
    payload = payload_;
  }
  table_->setRowCount(0);
  if (elapsed > 2.0) {
    status_->setText("Sin datos recientes de percepción");
    return;
  }
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(payload), &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    status_->setText("Mensaje de percepción inválido");
    return;
  }
  const auto root = document.object();
  const auto state = root.value("status").toString();
  const auto tables = root.value("tables");
  if (state != "ok" || !tables.isArray()) {
    status_->setText(state == "no_table" ? "Sin mesas u objetos observados" :
      "Percepción no disponible: " + state);
    return;
  }
  status_->setText("Frame: " + root.value("frame_id").toString());
  QJsonArray items;
  for (const auto & value : tables.toArray()) {
    if (!value.isObject()) {continue;}
    const auto table = value.toObject();
    items.append(table);
    for (const auto & object : table.value("objects").toArray()) {
      if (object.isObject()) {items.append(object);}
    }
  }
  const QMap<QString, QString> translations = {
    {"insufficient_history", "Faltan observaciones"}, {"position_variation", "Posición variable"},
    {"dimension_variation", "Tamaño variable"}, {"orientation_variation", "Orientación variable"},
    {"stale_observation", "Observación antigua"}};
  for (const auto & value : items) {
    const auto item = value.toObject();
    const double capture_age = item.value("age_sec").toDouble(-1);
    const double age = capture_age + elapsed;
    const bool fresh = capture_age >= 0 && std::isfinite(age) && age <= 0.5;
    const bool stable = fresh && item.value("stable").toBool(false);
    QStringList reasons;
    for (const auto & reason : item.value("stability").toObject().value("reasons").toArray()) {
      reasons.append(translations.value(reason.toString(), reason.toString()));
    }
    if (!fresh) {reasons.append("Observación antigua");}
    const auto dimensions = item.value("dimensions").toArray();
    QStringList sizes;
    for (const auto & size : dimensions) {sizes.append(QString::number(size.toDouble(), 'f', 3));}
    const auto depth = item.value("quality").toObject().value("valid_depth_fraction");
    const QStringList cells = {
      item.value("id").toString("Sin ID"),
      item.value("confirmed").toBool(false) ? "Confirmada" : "Provisional",
      !fresh ? "Antigua" : (stable ? "Estable" : "Inestable"),
      capture_age >= 0 ? QString::number(age, 'f', 2) : "—",
      sizes.join(" × "), depth.isDouble() ? QString::number(depth.toDouble()*100, 'f', 0)+"%" : "—",
      reasons.isEmpty() ? "—" : reasons.join(", ")};
    const int row = table_->rowCount();
    table_->insertRow(row);
    for (int column = 0; column < cells.size(); ++column) {
      auto * cell = new QTableWidgetItem(cells[column]);
      if (column == 2) {cell->setForeground(stable ? QColor(0, 160, 70) : QColor(200, 110, 0));}
      table_->setItem(row, column, cell);
    }
  }
}
}
PLUGINLIB_EXPORT_CLASS(g1_rviz_plugins::PerceptionPanel, rviz_common::Panel)
