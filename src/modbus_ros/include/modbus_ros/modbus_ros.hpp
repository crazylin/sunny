#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "shared_interfaces/msg/modbus_coord.hpp"

namespace modbus_ros
{

class ModbusRos : public rclcpp::Node
{
public:
    explicit ModbusRos(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ModbusRos();

private:
    void _Init();
    void _InitializeParameters();
    void _UpdateParameters();
    void _Sub(shared_interfaces::msg::ModbusCoord::UniquePtr ptr);

public:
    class _Impl;
    std::unique_ptr<_Impl> _impl;

    const char* _subName = "~/coord";
    rclcpp::Subscription<shared_interfaces::msg::ModbusCoord>::SharedPtr _sub;

    std::map<std::string, rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr> _map;

    std::thread _init;
};

}

