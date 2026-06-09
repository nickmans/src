#include "omni_rviz_controls/max_linear_velocity_panel.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

namespace omni_rviz_controls
{

namespace
{
constexpr int kSliderTicks = 200;
}

MaxLinearVelocityPanel::MaxLinearVelocityPanel(QWidget * parent)
: rviz_common::Panel(parent),
  value_label_(new QLabel(this)),
  status_label_(new QLabel(this)),
  slider_(new QSlider(Qt::Horizontal, this)),
  refresh_timer_(new QTimer(this)),
  target_node_name_("/waypoint_traj"),
  target_param_name_("max_linear_velocity_ms"),
  min_velocity_ms_(0.05),
  max_velocity_ms_(1.60),
  current_velocity_ms_(0.80),
  suppress_slider_callback_(false)
{
  auto * root_layout = new QVBoxLayout();
  auto * row_layout = new QHBoxLayout();

  auto * title_label = new QLabel("max_linear_velocity_ms", this);
  row_layout->addWidget(title_label);

  value_label_->setMinimumWidth(70);
  row_layout->addWidget(value_label_);
  row_layout->addStretch(1);

  slider_->setRange(0, kSliderTicks);
  slider_->setSingleStep(1);
  slider_->setPageStep(5);
  slider_->setTracking(true);
  slider_->setValue(velocityToSlider(current_velocity_ms_));

  root_layout->addLayout(row_layout);
  root_layout->addWidget(slider_);
  root_layout->addWidget(status_label_);
  setLayout(root_layout);

  connect(slider_, SIGNAL(valueChanged(int)), this, SLOT(onSliderValueChanged(int)));
  connect(slider_, SIGNAL(sliderReleased()), this, SLOT(onSliderReleased()));
  connect(refresh_timer_, SIGNAL(timeout()), this, SLOT(refreshTargetParameter()));

  onSliderValueChanged(slider_->value());
  setStatus("Waiting for /waypoint_traj...");
}

void MaxLinearVelocityPanel::onInitialize()
{
  auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setStatus("RViz ROS node unavailable.");
    return;
  }

  raw_node_ = node_abstraction->get_raw_node();
  if (!raw_node_) {
    setStatus("RViz ROS node unavailable.");
    return;
  }

  param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(raw_node_, target_node_name_);
  refresh_timer_->start(1000);
  refreshTargetParameter();
}

void MaxLinearVelocityPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("TargetNode", QString::fromStdString(target_node_name_));
  config.mapSetValue("TargetParam", QString::fromStdString(target_param_name_));
  config.mapSetValue("MinVelocity", min_velocity_ms_);
  config.mapSetValue("MaxVelocity", max_velocity_ms_);
  config.mapSetValue("CurrentVelocity", current_velocity_ms_);
}

void MaxLinearVelocityPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString target_node_qt;
  if (config.mapGetString("TargetNode", &target_node_qt) && !target_node_qt.isEmpty()) {
    target_node_name_ = target_node_qt.toStdString();
  }

  QString target_param_qt;
  if (config.mapGetString("TargetParam", &target_param_qt) && !target_param_qt.isEmpty()) {
    target_param_name_ = target_param_qt.toStdString();
  }

  float min_velocity = static_cast<float>(min_velocity_ms_);
  if (config.mapGetFloat("MinVelocity", &min_velocity)) {
    min_velocity_ms_ = static_cast<double>(min_velocity);
  }

  float max_velocity = static_cast<float>(max_velocity_ms_);
  if (config.mapGetFloat("MaxVelocity", &max_velocity)) {
    max_velocity_ms_ = static_cast<double>(max_velocity);
  }

  float current_velocity = static_cast<float>(current_velocity_ms_);
  if (config.mapGetFloat("CurrentVelocity", &current_velocity)) {
    current_velocity_ms_ = static_cast<double>(current_velocity);
  }

  min_velocity_ms_ = std::max(0.01, min_velocity_ms_);
  max_velocity_ms_ = std::max(min_velocity_ms_ + 0.01, max_velocity_ms_);
  current_velocity_ms_ = std::max(min_velocity_ms_, std::min(max_velocity_ms_, current_velocity_ms_));

  suppress_slider_callback_ = true;
  slider_->setValue(velocityToSlider(current_velocity_ms_));
  suppress_slider_callback_ = false;
  onSliderValueChanged(slider_->value());
}

void MaxLinearVelocityPanel::onSliderValueChanged(int slider_value)
{
  const double velocity = sliderToVelocity(slider_value);
  current_velocity_ms_ = velocity;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << velocity << " m/s";
  value_label_->setText(QString::fromStdString(oss.str()));

  if (!suppress_slider_callback_) {
    setStatus("Release slider to apply.");
  }
}

void MaxLinearVelocityPanel::onSliderReleased()
{
  if (!param_client_) {
    setStatus("Parameter client unavailable.");
    return;
  }

  if (!param_client_->service_is_ready()) {
    setStatus("/waypoint_traj parameter service not ready.");
    return;
  }

  const auto param = rclcpp::Parameter(target_param_name_, current_velocity_ms_);
  auto future = param_client_->set_parameters({param});

  if (future.wait_for(std::chrono::milliseconds(250)) != std::future_status::ready) {
    setStatus("Set request sent (timeout waiting for response).");
    return;
  }

  const auto results = future.get();
  if (!results.empty() && results.front().successful) {
    std::ostringstream oss;
    oss << "Applied " << target_param_name_ << "=" << std::fixed << std::setprecision(2)
        << current_velocity_ms_;
    setStatus(QString::fromStdString(oss.str()));
  } else if (!results.empty()) {
    setStatus(QString::fromStdString("Set failed: " + results.front().reason));
  } else {
    setStatus("Set failed: no response.");
  }
}

void MaxLinearVelocityPanel::refreshTargetParameter()
{
  if (!param_client_) {
    return;
  }

  if (!param_client_->service_is_ready()) {
    return;
  }

  auto future = param_client_->get_parameters({target_param_name_});
  if (future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    return;
  }

  auto values = future.get();
  if (values.empty()) {
    return;
  }

  const auto & value = values.front();
  if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return;
  }

  const double target_velocity = std::max(min_velocity_ms_, std::min(max_velocity_ms_, value.as_double()));

  if (std::fabs(target_velocity - current_velocity_ms_) < 1e-3) {
    return;
  }

  current_velocity_ms_ = target_velocity;

  suppress_slider_callback_ = true;
  slider_->setValue(velocityToSlider(current_velocity_ms_));
  suppress_slider_callback_ = false;

  onSliderValueChanged(slider_->value());
  setStatus("Synced from /waypoint_traj.");
}

double MaxLinearVelocityPanel::sliderToVelocity(int slider_value) const
{
  const double ratio = static_cast<double>(slider_value) / static_cast<double>(kSliderTicks);
  return min_velocity_ms_ + ratio * (max_velocity_ms_ - min_velocity_ms_);
}

int MaxLinearVelocityPanel::velocityToSlider(double velocity) const
{
  const double clamped = std::max(min_velocity_ms_, std::min(max_velocity_ms_, velocity));
  const double ratio = (clamped - min_velocity_ms_) / (max_velocity_ms_ - min_velocity_ms_);
  return static_cast<int>(std::lround(ratio * static_cast<double>(kSliderTicks)));
}

void MaxLinearVelocityPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

}  // namespace omni_rviz_controls

PLUGINLIB_EXPORT_CLASS(omni_rviz_controls::MaxLinearVelocityPanel, rviz_common::Panel)
