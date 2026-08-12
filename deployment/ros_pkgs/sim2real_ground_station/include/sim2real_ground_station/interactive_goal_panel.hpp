#ifndef SIM2REAL_GROUND_STATION_INTERACTIVE_GOAL_PANEL_HPP
#define SIM2REAL_GROUND_STATION_INTERACTIVE_GOAL_PANEL_HPP

#include <memory>
#include <mutex>

#include <QWidget>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <sim2real_planning_msgs/InteractiveGoalAction.h>

class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QTimer;

namespace sim2real_ground_station {

class InteractiveGoalPanel : public rviz::Panel {
  Q_OBJECT

 public:
  explicit InteractiveGoalPanel(QWidget* parent = nullptr);
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;

 Q_SIGNALS:
  void candidateForUi(double x, double y, double yaw_degrees);
  void statusForUi(const QString& text, bool error);
  void actionFinishedForUi(const QString& text, bool success);

 private Q_SLOTS:
  void onCandidateForUi(double x, double y, double yaw_degrees);
  void onStatusForUi(const QString& text, bool error);
  void onActionFinishedForUi(const QString& text, bool success);
  void onSendGoal();
  void refreshReadiness();

 private:
  using GoalClient = actionlib::SimpleActionClient<
      sim2real_planning_msgs::InteractiveGoalAction>;

  void candidateCallback(const geometry_msgs::PoseStampedConstPtr& message);
  void stateCallback(const mavros_msgs::StateConstPtr& message);
  void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr& message);
  void activeCallback();
  void feedbackCallback(
      const sim2real_planning_msgs::InteractiveGoalFeedbackConstPtr& feedback);
  void doneCallback(
      const actionlib::SimpleClientGoalState& state,
      const sim2real_planning_msgs::InteractiveGoalResultConstPtr& result);
  void showError(const QString& text);
  bool stateIsFresh(const ros::WallTime& received_at) const;

  ros::NodeHandle node_;
  ros::Subscriber candidate_subscriber_;
  ros::Subscriber state_subscriber_;
  ros::Subscriber extended_state_subscriber_;
  std::unique_ptr<GoalClient> action_client_;

  mutable std::mutex data_mutex_;
  geometry_msgs::PoseStamped candidate_;
  bool has_candidate_ = false;
  mavros_msgs::State state_;
  bool has_state_ = false;
  ros::WallTime state_received_at_;
  mavros_msgs::ExtendedState extended_state_;
  bool has_extended_state_ = false;
  ros::WallTime extended_state_received_at_;
  bool action_active_ = false;

  QLabel* endpoint_label_ = nullptr;
  QLabel* candidate_label_ = nullptr;
  QLabel* status_label_ = nullptr;
  QDoubleSpinBox* goal_height_ = nullptr;
  QDoubleSpinBox* takeoff_height_ = nullptr;
  QPushButton* send_button_ = nullptr;
  QTimer* readiness_timer_ = nullptr;
};

}  // namespace sim2real_ground_station

#endif
