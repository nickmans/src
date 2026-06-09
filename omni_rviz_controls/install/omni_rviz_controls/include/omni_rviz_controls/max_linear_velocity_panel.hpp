#ifndef OMNI_RVIZ_CONTROLS__MAX_LINEAR_VELOCITY_PANEL_HPP_
#define OMNI_RVIZ_CONTROLS__MAX_LINEAR_VELOCITY_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QSlider>
#include <QTimer>
#include <QWidget>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rviz_common/panel.hpp>

namespace omni_rviz_controls
{

class MaxLinearVelocityPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MaxLinearVelocityPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onSliderValueChanged(int slider_value);
  void onSliderReleased();
  void refreshTargetParameter();

private:
  double sliderToVelocity(int slider_value) const;
  int velocityToSlider(double velocity) const;
  void setStatus(const QString & text);

  QLabel * value_label_;
  QLabel * status_label_;
  QSlider * slider_;
  QTimer * refresh_timer_;

  rclcpp::Node::SharedPtr raw_node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> param_client_;

  std::string target_node_name_;
  std::string target_param_name_;
  double min_velocity_ms_;
  double max_velocity_ms_;
  double current_velocity_ms_;
  bool suppress_slider_callback_;
};

}  // namespace omni_rviz_controls

#endif  // OMNI_RVIZ_CONTROLS__MAX_LINEAR_VELOCITY_PANEL_HPP_
