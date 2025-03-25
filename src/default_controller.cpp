

#include <jh_franka_controllers/default_controller.h>

namespace jh_franka_controllers
{
// ================================================================================================
// ======================================== Core Functions ========================================
// ================================================================================================
bool DefaultController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle)
{
	std::vector<std::string> joint_names;
  std::string arm_id;
  ROS_WARN(
      "ForceExampleController: Make sure your robot's endeffector is in contact "
      "with a horizontal surface before starting the controller!");
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ForceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "ForceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "ForceExampleController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("ForceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("ForceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  
  mode_change_thread_ = std::thread(&DefaultController::modeChangeReaderProc, this);

  return true;
}

void DefaultController::starting(const ros::Time& time) 
{
  updateRobotData();
  start_time_ = time;
  q_init_ = q_;
  qdot_init_ = qdot_;
  q_desired_ = q_init_;
  qdot_desired_ = qdot_init_;
  x_init_ = x_;
  xdot_init_ = xdot_;
  rotation_init_ = rotation_;
}

void DefaultController::update(const ros::Time& time, const ros::Duration& period) 
{
  play_time_ = time;
  updateRobotData();

  if (calculation_mutex_.try_lock())
  {
      calculation_mutex_.unlock();
      if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
      async_calculation_thread_ = std::thread(&DefaultController::asyncCalculationProc, this);
  }
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1);
  while(std::chrono::steady_clock::now() < deadline)
  {
      if (calculation_mutex_.try_lock())
      {
          calculation_mutex_.unlock();
          if(async_calculation_thread_.joinable()) async_calculation_thread_.join();
          break;
      }
      std::this_thread::yield();
  }

  printRobotData();
  sendCmdInput();
}

// ================================================================================================
// ======================================= Thread Functions =======================================
// ================================================================================================
void DefaultController::modeChangeReaderProc()
{
   while (!quit_all_proc_)
    {
      if(kbhit())
      {
        {
          std::lock_guard<std::mutex> lock(calculation_mutex_);
          int key = getchar();
          switch (key)
          {
            case 'h':
              setMode(HOME);
              break;
            default:
              setMode(NONE);
              break;
          }
        }
      }
    }
}

void DefaultController::asyncCalculationProc()
{
  {
    std::lock_guard<std::mutex> lock(calculation_mutex_);
    if(is_mode_changed_)
    {
      is_mode_changed_ = false;
      control_start_time_ = play_time_;
      q_init_ = q_;
      qdot_init_ = qdot_;
      q_desired_ = q_init_;
      qdot_desired_ = qdot_init_;
      x_init_ = x_;
      xdot_init_ = xdot_;
      rotation_init_ = rotation_;
      if(control_mode_ == HOME)
      {
        std::cout << "======================== Mode cahnge: Home position ========================" << std::endl;
      }
  }

  if(control_mode_ == HOME)
  {
    Eigen::Vector7d target_q;
    target_q << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;
    q_desired_ = DyrosMath::cubicVector<7>(play_time_.toSec(), 
                                           control_start_time_.toSec(),
                                           control_start_time_.toSec() + 4.,
                                           q_init_,
                                           target_q,
                                           Eigen::Vector7d::Zero(),
                                           Eigen::Vector7d::Zero()
                                           );
    torque_desired_ = JointPDControl(q_desired_);
  }
  else
  {
    torque_desired_ = c_;
  }
  }
  calculation_cv_.notify_one();
}

// ================================================================================================
// ====================================== Utility Functions =======================================
// ================================================================================================	
void DefaultController::updateRobotData()
{
  const franka::RobotState &robot_state = state_handle_->getRobotState();
  const std::array<double, 42> &jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  const std::array<double, 7> &gravity_array = model_handle_->getGravity();
  const std::array<double, 49> &massmatrix_array = model_handle_->getMass();
  const std::array<double, 7> &coriolis_array = model_handle_->getCoriolis();
  {
    std::lock_guard<std::mutex> lock(robot_data_mutex_);
    q_ = Eigen::Map<const Eigen::Vector7d>(robot_state.q.data());
    qdot_ = Eigen::Map<const Eigen::Vector7d>(robot_state.dq.data());
    torque_ = Eigen::Map<const Eigen::Vector7d>(robot_state.tau_J.data());
    M_ = Eigen::Map<const Eigen::Matrix7d>(massmatrix_array.data());
    M_inv_ = M_.inverse();
    g_ = Eigen::Map<const Eigen::Vector7d>(gravity_array.data());
    c_ = Eigen::Map<const Eigen::Vector7d>(coriolis_array.data());

    Eigen::Affine3d transform;
    transform.matrix() = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());  
    x_ = transform.translation();
    rotation_ = transform.rotation();
    J_ = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(jacobian_array.data());
    xdot_ = J_ * qdot_;
    M_task_ = (J_ * M_inv_ * J_.transpose()).inverse();
    J_T_inv_ = M_task_ * J_ * M_inv_;
    g_task_ = J_T_inv_ * g_;
  }
}

void DefaultController::sendCmdInput()
{
  for (size_t i = 0; i < 7; ++i) 
  {
    joint_handles_[i].setCommand(torque_desired_(i));
  }
}

void DefaultController::printRobotData()
{
  if(print_rate_trigger_())
  {
    std::cout << "-------------------------------------------------------------------" << std::endl;
    std::cout << "MODE          : " << control_mode_ << std::endl;
    std::cout << "time          : " << std::fixed << std::setprecision(3) << play_time_.toSec() << std::endl;
		std::cout << "q now         :	";
		std::cout << std::fixed << std::setprecision(3) << q_.transpose() << std::endl;
		std::cout << "q desired     :	";
		std::cout << std::fixed << std::setprecision(3) << q_desired_.transpose() << std::endl;
    std::cout << "qdot now      :	";
		std::cout << std::fixed << std::setprecision(3) << qdot_.transpose() << std::endl;
    std::cout << "qdot desired  :	";
		std::cout << std::fixed << std::setprecision(3) << qdot_desired_.transpose() << std::endl;
    std::cout << "torque desired  :	";
		std::cout << std::fixed << std::setprecision(3) << torque_desired_.transpose() << std::endl;
		std::cout << "x             :	";
		std::cout << x_.transpose() << std::endl;
		std::cout << "R             :	" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << rotation_ << std::endl;
    std::cout << "J             :	" << std::endl;
		std::cout << std::fixed << std::setprecision(3) << J_ << std::endl;
  }
}

int DefaultController::kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if(ch != EOF)
	{
	ungetc(ch, stdin);
	return 1;
	}

	return 0;
}

void DefaultController::setMode(const CTRL_MODE& mode)
{
  is_mode_changed_ = true;
  control_mode_ = mode;
  std::cout << "Current mode (changed): " << mode << std::endl;
}

Eigen::Vector7d DefaultController::JointPDControl(const Eigen::Vector7d target_q)
{
  double kp, kv;
  kp = 1500;
  kv = 10;

  return M_ * (kp * (target_q - q_) + kv * (-qdot_)) + c_;
}
} // namespace jh_franka_controllers



PLUGINLIB_EXPORT_CLASS(jh_franka_controllers::DefaultController,
                       controller_interface::ControllerBase)

