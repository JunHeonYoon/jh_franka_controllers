

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <mutex>

#include <controller_interface/controller_base.h>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka/robot_state.h>
#include <franka_hw/trigger_rate.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <realtime_tools/realtime_publisher.h>
#include <pluginlib/class_list_macros.h>

#include "math_type_define.h"
#include "suhan_benchmark.h"


namespace jh_franka_controllers 
{

class DefaultController : public controller_interface::MultiInterfaceController<
								   franka_hw::FrankaModelInterface,
								   hardware_interface::EffortJointInterface,
								   franka_hw::FrankaStateInterface> {
                     
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private: 
  // ========================================================================
  // ========================= Franka robot handles =========================
  // ========================================================================
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // ========================================================================
  // =========================== Franka robot Data ==========================
  // ========================================================================
  // ====== Joint space data ======
  // initial state
  Eigen::Vector7d q_init_;
  Eigen::Vector7d qdot_init_;

  // current state
  Eigen::Vector7d q_;
  Eigen::Vector7d qdot_;
  Eigen::Vector7d torque_;

  // control value
  Eigen::Vector7d q_desired_;
  Eigen::Vector7d qdot_desired_;
  Eigen::Vector7d torque_desired_;

  // Dynamics
  Eigen::Matrix7d M_;
  Eigen::Matrix7d M_inv_;
  Eigen::Vector7d c_;
  Eigen::Vector7d g_;

  // ====== Task space data =======
  // initial state
  Eigen::Vector3d x_init_;
  Eigen::Vector6d xdot_init_;
  Eigen::Matrix3d rotation_init_;

  // current state
  Eigen::Vector3d x_;
  Eigen::Vector6d xdot_;
  Eigen::Matrix3d rotation_;
  Eigen::Matrix<double, 6, 7> J_;

  // Dynamics
  Eigen::Matrix6d M_task_;
  Eigen::Vector6d g_task_;
  Eigen::Matrix<double, 6, 7> J_T_inv_;

  // ========================================================================
  // ============================ Mutex & Thread ============================
  // ========================================================================
  std::mutex robot_data_mutex_;
  std::mutex calculation_mutex_;
  bool quit_all_proc_ {false};
  std::thread async_calculation_thread_;
  std::thread mode_change_thread_;
  std::thread state_pub_thread_;
  std::condition_variable calculation_cv_;

  // ========================================================================
  // =========================== Controller data ============================
  // ========================================================================
  const double hz_{1000.};
  ros::Time start_time_;
  ros::Time play_time_;
  ros::Time control_start_time_;

  enum CTRL_MODE{NONE, HOME};
  CTRL_MODE control_mode_{NONE};
  bool is_mode_changed_ {false};

  franka_hw::TriggerRate print_rate_trigger_{10}; 

  SuhanBenchmark bench_timer_;

  // ========================================================================
  // =========================== Thread Functions ===========================
  // ========================================================================	
  void modeChangeReaderProc();
  void asyncCalculationProc();

  // ========================================================================
  // ========================== Utility Functions ===========================
  // ========================================================================					
  void updateRobotData();
  void sendCmdInput();
  void printRobotData();
  int kbhit(void);
  void setMode(const CTRL_MODE& mode);
  Eigen::Vector7d JointPDControl(const Eigen::Vector7d target_q);
};

}  // namespace jh_franka_controllers
