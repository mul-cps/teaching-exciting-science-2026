#include "so101_station_operator/operator_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QJsonDocument>
#include <QJsonObject>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QMessageBox>
#include <QScrollArea>
#include <chrono>
#include <algorithm>
#include <rviz_common/display_context.hpp>
#include <rviz_common/display_group.hpp>

namespace so101_station_operator {
OperatorPanel::OperatorPanel(QWidget * parent) : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout(this);
  for (const auto & entry : std::vector<std::pair<QString, QString>>{
      {"Restart entire station", "emergency_reset"}, {"Restart MoveIt", "restart_moveit"},
      {"ZERO TORQUE / STOP", "zero_torque"}}) {
    auto button = new QPushButton(entry.first, this);
    button->setProperty("command", entry.second);
    button->setMinimumHeight(34);
    button->setStyleSheet(entry.second == "zero_torque" ? "font-weight: bold; color: #b00020;" : "font-weight: bold;");
    connect(button, &QPushButton::clicked, this, [this, command = entry.second] { request(command); });
    buttons_.push_back(button);
    layout->addWidget(button);
  }
  status_ = new QLabel("Waiting for operator API measured status…", this);
  status_->setWordWrap(true);
  layout->addWidget(status_);
  auto grid = new QGridLayout;
  const std::vector<std::pair<QString, QString>> commands = {
    {"Reset (torque OFF)", "reset"}, {"Saved home position", "start"},
    {"Release for setup", "base_release"}, {"Record reference", "base_confirm"},
    {"Leader teleop", "teleop"}, {"MoveIt mode", "moveit"},
    {"Teleop: relative", "teleop_relative"}, {"Teleop: absolute", "teleop_absolute"},
    {"Damping / renew 60 s", "damping"},
    {"Restart camera", "restart_camera"}, {"Restart depth", "restart_depth"},
    {"Restart tags", "restart_tags"}, {"Restart perception", "restart_all"},
    {"Verify restart / recover", "verify_restart"}};
  for (size_t i = 0; i < commands.size(); ++i) {
    auto button = new QPushButton(commands[i].first, this);
    button->setProperty("command", commands[i].second);
    connect(button, &QPushButton::clicked, this, [this, command = commands[i].second] { request(command); });
    if (commands[i].second == "damping") button->setToolTip("Manual backdrivable resistance. Requires enabled exact firmware capability; expires to torque OFF after 60 s.");
    buttons_.push_back(button);
    grid->addWidget(button, int(i / 2), int(i % 2));
  }
  auto controls = new QWidget(this);
  controls->setLayout(grid);
  auto scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setWidget(controls);
  scroll->setMinimumHeight(80);
  layout->addWidget(scroll, 1);
  result_ = new QLabel("Commands use the resident owner; no direct hardware access.", this);
  result_->setWordWrap(true);
  layout->addWidget(result_);
  node_ = std::make_shared<rclcpp::Node>("so101_operator_panel", rclcpp::NodeOptions().use_global_arguments(false));
  executor_.add_node(node_);
  status_sub_ = node_->create_subscription<std_msgs::msg::String>("/station/operator/status", 1,
    [this](const std_msgs::msg::String::SharedPtr message) {
      const auto document = QJsonDocument::fromJson(QByteArray::fromStdString(message->data));
      if (!document.isObject()) return;
      const auto object = document.object();
      fresh_ = object.value("fresh").toBool(false);
      damping_supported_ = object.value("damping_supported").toBool(false);
      capabilities_ = object.value("capabilities").toObject();
      api_uncertain_ = object.value("outcome_unknown").toBool(false);
      status_->setText(object.value("summary").toString("Invalid operator status"));
      received_.restart();
    });
  connect(&timer_, &QTimer::timeout, this, &OperatorPanel::tick);
  timer_.start(20);
}
OperatorPanel::~OperatorPanel() { timer_.stop(); executor_.remove_node(node_); }
void OperatorPanel::request(const QString & command) {
  if (command == "base_release" || command == "reset" || command == "emergency_reset" || command == "restart_moveit") {
    if (QMessageBox::question(this, "Release arm torque", "Support the arm before releasing torque. Continue?") != QMessageBox::Yes) return;
  }
  if (command == "damping" && QMessageBox::question(this, "Manual damping lease", "Hold/support the arm. Enable or renew gentle backdrivable resistance for 60 seconds? Torque switches OFF when the lease expires.") != QMessageBox::Yes) return;
  for (const auto & active : requests_) if (active.command == command && command != "zero_torque") return;
  if (pending_ && command != "zero_torque" && command != "emergency_reset") return;
  auto client = node_->create_client<std_srvs::srv::Trigger>(("/station/operator/" + command).toStdString());
  if (!client->service_is_ready()) { result_->setText("Operator service unavailable"); return; }
  Request request;
  request.command = command;
  request.client = client;
  request.future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>()).future;
  request.elapsed.start();
  requests_.push_back(std::move(request));
  pending_ = true;
  result_->setText(command + ": pending; acknowledgement is required");
}
void OperatorPanel::tick() {
  executor_.spin_some(std::chrono::milliseconds(2));
  for (auto it = requests_.begin(); it != requests_.end();) {
    if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto result = it->future.get();
        QString model;
        if (result->success && (it->command == "restart_moveit" || it->command == "emergency_reset")) {
          int reloaded = 0;
          auto context = getDisplayContext();
          if (context) {
            const auto reload = [&reloaded](auto && self, rviz_common::DisplayGroup * group) -> void {
              for (int i = 0; i < group->numDisplays(); ++i) {
                auto display = group->getDisplayAt(i);
                if (display->getClassId() == "moveit_rviz_plugin/MotionPlanning" && display->isEnabled()) {
                  display->setEnabled(false);
                  display->setEnabled(true);
                  ++reloaded;
                }
                if (auto child = qobject_cast<rviz_common::DisplayGroup *>(display)) self(self, child);
              }
            };
            reload(reload, context->getRootDisplayGroup());
          }
          model = reloaded ? "; RViz planning model reloaded at current measured pose; no motion requested" :
            "; RViz planning display unavailable: reload MotionPlanning manually before planning";
        }
        result_->setText(it->command + (result->success ? ": confirmed: " : ": rejected: ") + QString::fromStdString(result->message) + model);
      } catch (const std::exception & error) {
        uncertain_ = true;
        result_->setText(it->command + ": service failed: " + error.what());
      }
      it = requests_.erase(it);
    } else if (it->elapsed.elapsed() > (it->command == "emergency_reset" ? 125000 : 110000)) {
      result_->setText(it->command + ": response timed out; outcome unknown. Inspect status. STOP remains available.");
      uncertain_ = true;
      it = requests_.erase(it);
    } else { ++it; }
  }
  pending_ = !requests_.empty();
  const bool live = received_.isValid() && received_.elapsed() < 2000;
  if (!live) { fresh_ = false; status_->setText("Operator API status is stale / unavailable"); }
  for (auto button : buttons_) {
    auto command = button->property("command").toString();
    const auto capability = capabilities_.value(command).toObject();
    const bool supported = capability.value("enabled").toBool(false);
    auto tooltip = capability.value("reason").toString();
    if (command == "restart_moveit" || command == "emergency_reset") tooltip += "\nRestarts at the current measured pose with torque OFF. No encoder zero, calibration change or home motion.";
    if (command == "teleop_relative") tooltip += "\nRelative teleop anchors at the current pose. Select only while teleop is disabled.";
    if (command == "teleop_absolute") tooltip += "\nAbsolute teleop follows calibrated leader targets directly with native acceleration smoothing. Align the leader before enabling.";
    button->setToolTip(tooltip);
    button->setEnabled(command == "zero_torque" || (live && command == "emergency_reset" && supported && std::none_of(requests_.begin(), requests_.end(), [](const Request & request) { return request.command == "emergency_reset"; })) || (live && !pending_ && supported && command == "verify_restart") ||
      (live && !pending_ && !uncertain_ && !api_uncertain_ && supported && (fresh_ || command == "reset" || command == "emergency_reset")));
  }
}
}
PLUGINLIB_EXPORT_CLASS(so101_station_operator::OperatorPanel, rviz_common::Panel)
