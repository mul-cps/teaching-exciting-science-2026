#include "so101_station_operator/ev3_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <QJsonDocument>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QMessageBox>
#include <chrono>
namespace so101_station_operator {
EV3Panel::EV3Panel(QWidget * parent) : rviz_common::Panel(parent) {
  auto layout = new QVBoxLayout(this);
  status_ = new QLabel("EV3 workflow API unavailable; connect explicitly.", this);
  status_->setWordWrap(true); layout->addWidget(status_);
  auto grid = new QGridLayout;
  const std::vector<std::pair<QString, QString>> actions = {
    {"Connect EV3", "connect"}, {"Conveyor home (switch)", "home"},
    {"Conveyor left end", "left_end"}, {"Conveyor right end", "right_end"},
    {"Eject one brick", "eject"}, {"STOP station / EV3", "stop"},
    {"Record feeder eject reference", "confirm_feeder"}, {"Wake EV3", "wake"}, {"Sleep EV3", "sleep"}, {"Reset inspected fault", "reset_fault"}};
  for (size_t i = 0; i < actions.size(); ++i) {
    auto button = new QPushButton(actions[i].first, this);
    buttons_[actions[i].second] = button;
    button->setEnabled(actions[i].second == "stop");
    connect(button, &QPushButton::clicked, this, [this, command=actions[i].second] { request(command); });
    grid->addWidget(button, int(i/2), int(i%2));
  }
  layout->addLayout(grid);
  setup_ = new QLabel("Endpoint mapping and feeder reference status will appear after connection.", this);
  setup_->setWordWrap(true); layout->addWidget(setup_);
  result_ = new QLabel("No motion on panel load or connection. Endpoints require commissioned direction mapping.", this);
  result_->setWordWrap(true); layout->addWidget(result_);
  node_ = std::make_shared<rclcpp::Node>("ev3_operations_panel", rclcpp::NodeOptions().use_global_arguments(false));
  executor_.add_node(node_);
  reference_client_ = node_->create_client<so101_station_workflow::srv::ConfirmReference>("/station/ev3/confirm_reference");
  subscription_ = node_->create_subscription<std_msgs::msg::String>("/station/ev3/operator_status", 1,
    [this](const std_msgs::msg::String::SharedPtr message) { receive(message->data); });
  connect(&timer_, &QTimer::timeout, this, &EV3Panel::tick); timer_.start(50);
}
EV3Panel::~EV3Panel() { timer_.stop(); executor_.remove_node(node_); }
void EV3Panel::receive(const std::string & data) {
  if (data.size() > 8192) return;
  auto document = QJsonDocument::fromJson(QByteArray::fromStdString(data));
  if (!document.isObject()) return;
  auto object = document.object();
  if (object.value("schema").toInt() != 1) return;
  capabilities_ = object.value("capabilities").toObject();
  received_.restart();
  auto angles = object.value("angles").toObject();
  auto refs = object.value("references").toObject();
  status_->setText(QString("Connection: %1 | telemetry: %2 | %3 | power: %4\nConveyor D: %5° | home: %6 | S1: %7\nFeeder A: %8° | reference: %9 | color S3: %10\nFault: %11 | STOP pending: %12")
    .arg(object.value("connected").toBool() ? "connected" : "disconnected")
    .arg(object.value("fresh").toBool() ? "fresh" : "STALE")
    .arg(object.value("busy").toBool() ? "BUSY" : "idle")
    .arg(object.value("power_mode").toString())
    .arg(angles.value("sorter").isDouble() ? QString::number(angles.value("sorter").toDouble()) : "unknown")
    .arg(refs.value("sorter").toString("unconfirmed"))
    .arg(object.value("touch").isBool() ? (object.value("touch").toBool() ? "pressed" : "released") : "unknown")
    .arg(angles.value("feeder").isDouble() ? QString::number(angles.value("feeder").toDouble()) : "unknown")
    .arg(refs.value("feeder").toString("unconfirmed"))
    .arg(object.value("color").toString()).arg(object.value("fault").toString("none"))
    .arg(object.value("stop_pending").toBool() ? "yes" : "no"));
  setup_->setText(QString("%1\nLeft: %2\nRight: %3")
    .arg(refs.value("feeder").toString() == "eject" ? "Feeder EJECT reference recorded for this session" :
      "Feeder EJECT reference MISSING: manually place pusher fully ejected, then Record feeder eject reference")
    .arg(capabilities_.value("left_end").toObject().value("reason").toString())
    .arg(capabilities_.value("right_end").toObject().value("reason").toString()));
  auto workflow = object.value("workflow").toObject();
  if (!active_job_.isEmpty() && workflow.value("job_id").toString() == active_job_) {
    const auto state = workflow.value("state").toString();
    if (state == "idle" || state == "fault") {
      result_->setText(active_command_ + (state == "idle" ? ": completed and verified by owner" : ": FAILED: " + workflow.value("fault").toString()));
      active_job_.clear();
    }
  }
}
void EV3Panel::request(const QString & command) {
  if (pending_.count(command)) return;
  if (command != "stop" && (!pending_.empty() || reference_pending_ || !active_job_.isEmpty() || uncertain_)) return;
  if (command == "confirm_feeder") {
    if (QMessageBox::question(this, "Record physical feeder EJECT reference",
        "Verify the mechanism is stopped and the pusher is manually positioned fully EJECTED. "
        "This records the current feeder position as zero; it does not home the feeder. "
        "Have you physically verified this exact EJECT position?",
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) {
      result_->setText("Feeder reference unchanged: physical confirmation declined"); return;
    }
    auto client = reference_client_;
    if (!client->service_is_ready()) { result_->setText("Reference service unavailable; nothing recorded"); return; }
    auto request = std::make_shared<so101_station_workflow::srv::ConfirmReference::Request>();
    request->name = "feeder"; request->reference = "eject"; request->physically_verified = true;
    ReferencePending pending; pending.client = client;
    auto sent = client->async_send_request(request);
    pending.future = sent.future.share(); pending.request_id = sent.request_id; pending.elapsed.start();
    reference_pending_.emplace(std::move(pending));
    result_->setText("Record feeder EJECT reference: pending typed service acknowledgement"); return;
  }
  const auto path = command == "connect" || command == "home" || command == "eject" || command == "left_end" || command == "right_end" ? "/station/ev3/"+command : "/station/"+command;
  auto client = node_->create_client<std_srvs::srv::Trigger>(path.toStdString());
  if (!client->service_is_ready()) { result_->setText(command + ": service unavailable"); return; }
  Pending pending;
  pending.client = client;
  auto sent = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  pending.future = sent.future.share(); pending.request_id = sent.request_id;
  pending.elapsed.start(); pending_.emplace(command, std::move(pending));
  result_->setText(command + ": pending acknowledgement");
}
void EV3Panel::acknowledge(const QString & command, bool success, const QString & message) {
  auto object = QJsonDocument::fromJson(message.toUtf8()).object();
  if (success && object.value("job_id").isString() && command != "stop") {
    active_job_ = object.value("job_id").toString(); active_command_ = command; job_elapsed_.restart();
    result_->setText(command + ": accepted; awaiting terminal job acknowledgement");
  } else result_->setText(command + (success ? ": acknowledged: " : ": rejected: ") + message.left(1024));
}
void EV3Panel::tick() {
  executor_.spin_some(std::chrono::milliseconds(2));
  for (auto it = pending_.begin(); it != pending_.end();) {
    if (it->second.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto response = it->second.future.get();
        acknowledge(it->first, response->success, QString::fromStdString(response->message));
      } catch (const std::exception & error) { uncertain_ = true; result_->setText(QString("Outcome unknown: ")+error.what()); }
      it = pending_.erase(it);
    } else if (it->second.elapsed.elapsed() > 5000) {
      it->second.client->remove_pending_request(it->second.request_id);
      uncertain_ = true; result_->setText(it->first + ": timeout; outcome unknown. STOP remains available; inspect owner status.");
      it = pending_.erase(it);
    } else ++it;
  }
  if (reference_pending_) {
    auto & pending = *reference_pending_;
    if (pending.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        auto response = pending.future.get();
        acknowledge("confirm_feeder", response->success, QString::fromStdString(response->message));
      } catch (const std::exception & error) { uncertain_ = true; result_->setText(QString("Reference outcome unknown: ")+error.what()); }
      reference_pending_.reset();
    } else if (pending.elapsed.elapsed() > 5000) {
      pending.client->remove_pending_request(pending.request_id); reference_pending_.reset(); uncertain_ = true;
      result_->setText("Reference acknowledgement timed out; outcome unknown. Inspect owner status. STOP remains available.");
    }
  }
  if (!active_job_.isEmpty() && job_elapsed_.elapsed() > 30000) {
    uncertain_ = true; active_job_.clear();
    result_->setText("Terminal job acknowledgement timed out; outcome unknown. STOP remains available; inspect owner status.");
  }
  bool live = received_.isValid() && received_.elapsed() < 2000;
  if (!live) status_->setText("EV3 operator status STALE / unavailable. STOP remains available.");
  for (const auto & entry : buttons_) {
    auto capability = capabilities_.value(entry.first).toObject();
    entry.second->setToolTip(capability.value("reason").toString());
    entry.second->setEnabled(entry.first == "stop" ? !pending_.count("stop") :
      live && pending_.empty() && !reference_pending_ && active_job_.isEmpty() && !uncertain_ && capability.value("enabled").toBool());
  }
}
}
PLUGINLIB_EXPORT_CLASS(so101_station_operator::EV3Panel, rviz_common::Panel)
