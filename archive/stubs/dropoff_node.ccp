#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/gripper_command.hpp"

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

using GripperCommand = control_msgs::action::GripperCommand;
using std::placeholders::_1;

// ── Gripper positions ─────────────────────────────────────────────────────
static constexpr double GRIPPER_OPEN = 0.2102;

// ── Poses ─────────────────────────────────────────────────────────────────
static constexpr double HOME_X = 0.23, HOME_Y = 0.0, HOME_Z = 0.33;
static constexpr double DROP_X = 0.25, DROP_Y = 0.0, DROP_Z = 0.10;

static constexpr double MOVE_DURATION = 6.0;


class DropoffNode : public rclcpp::Node
{
public:
  DropoffNode() : Node("arm_node_dropoff"), active_(false), ik_ready_(false)
  {
    // ── URDF subscription (latched) ───────────────────────────────────────
    urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1))
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&DropoffNode::urdfCb, this, _1));

    // ── Activation: "start" from dropoff motor node ───────────────────────
    activate_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/activate_arm/drop", 10,
      std::bind(&DropoffNode::activateCb, this, _1));

    // ── Joint states ──────────────────────────────────────────────────────
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&DropoffNode::jointStateCb, this, _1));

    // ── Publishers ────────────────────────────────────────────────────────
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/mirte_master_arm_controller/joint_trajectory", 10);

    drop_done_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/block_dropped_off", 10);

    // ── Gripper action client ─────────────────────────────────────────────
    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "/mirte_master_gripper_controller/gripper_cmd");

    RCLCPP_INFO(get_logger(), "DropoffNode ready — waiting for activation.");
  }

private:

  void urdfCb(const std_msgs::msg::String & msg)
  {
    kdl_parser::treeFromString(msg.data, tree_);
    tree_.getChain("base_link", "wrist", chain_);
    solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
    ik_ready_ = true;
    RCLCPP_INFO(get_logger(), "IK solver ready (%u joints).", chain_.getNrOfJoints());
  }

  void activateCb(const std_msgs::msg::String & msg)
  {
    if (msg.data == "start" && !active_) {
      active_ = true;
      RCLCPP_INFO(get_logger(), "Dropoff ACTIVATED.");
      executeDrop();
    } else if (msg.data == "stop") {
      active_ = false;
    }
  }

  void jointStateCb(const sensor_msgs::msg::JointState & msg)
  {
    static const std::vector<std::string> names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"
    };
    current_joints_.clear();
    for (const auto & n : names) {
      auto it = std::find(msg.name.begin(), msg.name.end(), n);
      if (it != msg.name.end())
        current_joints_.push_back(msg.position[std::distance(msg.name.begin(), it)]);
    }
  }

  bool solveIK(double x, double y, double z,
               double qx, double qy, double qz, double qw,
               std::vector<double> & result)
  {
    if (!ik_ready_) { RCLCPP_WARN(get_logger(), "IK not ready."); return false; }

    KDL::Frame target(
      KDL::Rotation::Quaternion(qx, qy, qz, qw),
      KDL::Vector(x, y, z));

    KDL::JntArray q_init(chain_.getNrOfJoints());
    if (current_joints_.size() == chain_.getNrOfJoints())
      for (unsigned i = 0; i < chain_.getNrOfJoints(); ++i)
        q_init(i) = current_joints_[i];

    KDL::JntArray q_out(chain_.getNrOfJoints());
    int status = solver_->CartToJnt(q_init, target, q_out);
    if (status < 0) {
      RCLCPP_ERROR(get_logger(), "IK failed (status %d).", status);
      return false;
    }
    result.clear();
    for (unsigned i = 0; i < 4 && i < chain_.getNrOfJoints(); ++i)
      result.push_back(q_out(i));
    return true;
  }

  void sendJoints(const std::vector<double> & positions, double duration_sec)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"
    };
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = positions;
    pt.time_from_start.sec     = static_cast<int32_t>(duration_sec);
    pt.time_from_start.nanosec = 0;
    traj.points.push_back(pt);
    arm_pub_->publish(traj);
    RCLCPP_INFO(get_logger(), "Arm moving to [%.2f, %.2f, %.2f, %.2f]",
      positions[0], positions[1], positions[2], positions[3]);
    rclcpp::sleep_for(std::chrono::milliseconds(
      static_cast<int>((duration_sec + 0.5) * 1000)));
  }

  bool sendPose(double x, double y, double z,
                double qx = 1.0, double qy = 0.0, double qz = 0.0, double qw = 0.0)
  {
    std::vector<double> joints;
    if (!solveIK(x, y, z, qx, qy, qz, qw, joints)) return false;
    sendJoints(joints, MOVE_DURATION);
    return true;
  }

  void openGripper()
  {
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_WARN(get_logger(), "Gripper action server not available.");
      return;
    }
    auto goal = GripperCommand::Goal();
    goal.command.position   = GRIPPER_OPEN;
    goal.command.max_effort = 10.0;
    gripper_client_->async_send_goal(goal);
    rclcpp::sleep_for(std::chrono::milliseconds(1500));
  }

  void executeDrop()
  {
    RCLCPP_INFO(get_logger(), "Step 1: move to drop position");
    if (!sendPose(DROP_X, DROP_Y, DROP_Z)) {
      RCLCPP_ERROR(get_logger(), "Drop IK failed — aborting.");
      active_ = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Step 2: open gripper");
    openGripper();

    RCLCPP_INFO(get_logger(), "Step 3: retract to home");
    sendPose(HOME_X, HOME_Y, HOME_Z);

    RCLCPP_INFO(get_logger(), "Drop complete — publishing /block_dropped_off");
    std_msgs::msg::String done;
    done.data = "dropped_off";
    drop_done_pub_->publish(done);

    active_ = false;
  }

  bool active_;
  bool ik_ready_;
  std::vector<double> current_joints_;

  KDL::Tree  tree_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        urdf_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        activate_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                 drop_done_pub_;

  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DropoffNode>());
  rclcpp::shutdown();
  return 0;
}