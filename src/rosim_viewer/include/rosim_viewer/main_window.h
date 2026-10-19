#pragma once

#include <map>

#include <QString>
#include <QMainWindow>
#include <QTimer>

#include <Eigen/Geometry>

#include <sensor_msgs/msg/joint_state.hpp>

#include "rosim_viewer/ros_bridge.h"
#include "rosim_viewer/robot_assembly.h"
#include "rosim_viewer/scene_view.h"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSlider;
class QTreeWidget;
class QTreeWidgetItem;

namespace rosim::viewer
{

/// \brief Main application window: left control panel + central 3D view.
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow() override;

private slots:
  void onImportClicked();
  void onImportFinished(bool ok, QString model_id, QString message, int edge_count);
  void onFollowClicked();
  void onStopClicked();
  void onSpeedChanged(int value);
  void onEdgeSelectedInScene(int index);
  void onTreeSelectionChanged();
  void onFollowFeedback(float progress, QString phase);
  void onFollowResult(bool ok, QString message, double cartesian_fraction);
  void onJointState(sensor_msgs::msg::JointState state);
  void onHomeClicked();
  void onGoHomeResult(bool ok, QString message);
  void onJogResult(bool ok, QString message);

private:
  void buildUi();
  void log(const QString& line);
  void setBusy(bool busy);

  RosBridge* bridge_ = nullptr;
  SceneView* scene_ = nullptr;
  RobotAssembly* robot_ = nullptr;

  QString current_model_;
  int selected_edge_ = -1;
  QString sample_step_path_;
  /// model_id -> local STEP path (for the viewer's own solid tessellation)
  std::map<QString, QString> model_paths_;
  /// path chosen in the file dialog, awaiting the import service result
  QString pending_model_path_;
  Eigen::Isometry3d pending_pose_ = Eigen::Isometry3d::Identity();

  // controls
  QTreeWidget* tree_ = nullptr;
  QComboBox* orient_combo_ = nullptr;
  QDoubleSpinBox* place_x_ = nullptr;
  QDoubleSpinBox* place_y_ = nullptr;
  QDoubleSpinBox* place_z_ = nullptr;
  QDoubleSpinBox* place_yaw_ = nullptr;
  QDoubleSpinBox* tool_offset_spin_ = nullptr;
  QDoubleSpinBox* approach_spin_ = nullptr;
  QDoubleSpinBox* spacing_spin_ = nullptr;
  QDoubleSpinBox* tilt_spin_ = nullptr;
  QSlider* speed_slider_ = nullptr;
  QLabel* speed_label_ = nullptr;
  QPushButton* import_btn_ = nullptr;
  QPushButton* follow_btn_ = nullptr;
  QPushButton* stop_btn_ = nullptr;
  QPushButton* home_btn_ = nullptr;
  QDoubleSpinBox* jog_step_spin_ = nullptr;
  /// joint name -> live angle readout next to its jog buttons
  std::map<QString, QLabel*> jog_value_labels_;
  /// one manual request (home/jog) in flight — blocks further clicks
  bool manual_pending_ = false;
  QProgressBar* progress_ = nullptr;
  QLabel* phase_label_ = nullptr;
  QPlainTextEdit* log_box_ = nullptr;

  /// render throttle: joint states up to 100 Hz, renders at ~33 Hz
  QTimer render_timer_;
  bool render_dirty_ = false;
};

}  // namespace rosim::viewer
