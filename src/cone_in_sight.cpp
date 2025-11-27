#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>



class ConeInSight : public BT::ConditionNode
{
public:
  ConeInSight(const std::string &name, const BT::NodeConfiguration &config)
  : BT::ConditionNode(name, config), detected_(false)
  {
    if (!config.blackboard->get("node", node_)) {
    throw BT::RuntimeError("ConeInSight: missing ROS2 node on blackboard");
}

    cb_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_.add_callback_group(cb_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cb_group_;


    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/cone_detected",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&ConeInSight::coneCallback, this, std::placeholders::_1),
        opts);

    std::cout << "[ConeInSight] Subscribed to /cone_detected\n";
  }

  static BT::PortsList providedPorts()
  {
    return{};
  }

  BT::NodeStatus tick() override
  {
    executor_.spin_some();

    return detected_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:

  void coneCallback(std_msgs::msg::Bool::SharedPtr msg)
  {
    detected_ = msg->data;
    std::cout << "[ConeInSight] msg = " << (detected_ ? "Detected" : "Not Detected") << "\n";
  }


  bool detected_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<ConeInSight>(name, config);
    };

  factory.registerBuilder<ConeInSight>("ConeInSight", builder);
}



