#include "modbus_ros/modbus_ros.hpp"
#include <climits>

extern "C"
{
    #include <errno.h>
    #include <modbus.h>
    #include <unistd.h>
}

using namespace std::chrono_literals;

namespace modbus_ros
{

bool EndsWith(const std::string& value, const std::string& ending)
{
    if(value.size() < ending.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

void CheckAndSend(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr ptr)
{
    auto t = std::make_shared<std_srvs::srv::Trigger::Request>();

    if(rclcpp::ok() && ptr->service_is_ready())
        ptr->async_send_request(t);
    //else
        //throw std::runtime_error("Service failed");
}

class ModbusRos::_Impl
{
public:
    explicit _Impl(ModbusRos* ptr) : _node(ptr)
    {
        ctx = modbus_new_tcp(NULL, 502);
        if(!ctx)
            throw std::runtime_error("Can not create modbus context");

        header_length = modbus_get_header_length(ctx);

        mb_mapping = modbus_mapping_new(0, 0, 40, 0);
        if(!mb_mapping)
        {
            modbus_free(ctx);
            throw std::runtime_error("Can not initialize modbus registers");
        }

        std::thread([this]()
        {
            while(rclcpp::ok())
            {
                try
                {
                    _ListenAndAccept();
                    _Receive();
                }
                catch(const std::exception& e)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros workflow: %s", e.what());
                }
                catch(...)
                {
                    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros workflow: unknown");
                }
            }
        }).detach();
    }

    ~_Impl()
    {
        close(sock);
        modbus_mapping_free(mb_mapping);
        modbus_close(ctx);
        modbus_free(ctx);
    }

    void Update(bool valid, double x = 0, double y = 0, double z = 0)
    {
        std::lock_guard<std::mutex> lock(_mutex);
        if(z * 10000 > SHRT_MAX)
        {
            mb_mapping->tab_registers[16] = 0;
            mb_mapping->tab_registers[17] = 0;
            mb_mapping->tab_registers[18] = 0;
            mb_mapping->tab_registers[19] = 0;
        }
        else
        {
            mb_mapping->tab_registers[16] = (uint16_t) valid;
            mb_mapping->tab_registers[17] = (uint16_t) x;
            mb_mapping->tab_registers[18] = (uint16_t) y * 10000;
            mb_mapping->tab_registers[19] = (uint16_t) z * 10000;
        }
    }

private:
    void _ListenAndAccept()
    {
        while(rclcpp::ok())
        {
            close(sock);
            sock = modbus_tcp_listen(ctx, 1);
            if(sock != -1 && modbus_tcp_accept(ctx, &sock) != -1)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "modbus_ros initialized listen and accept successfully");
                return;
            }
            
            std::this_thread::sleep_for(200ms);
        }
    }
    
    void _Receive()
    {
        while(rclcpp::ok())
        {
            int rc;
            do
            {
                rc = modbus_receive(ctx, query);
            }
            while(rc == 0 && rclcpp::ok());
            
            if(rc <= 0)
                break;
                
            if(query[7] == 0x06 && query[9] == 0x01)//laser control
            {
                if(query[11])
                    CheckAndSend(_node->_map["/camera_spinnaker_node/start"]);
                else
                    CheckAndSend(_node->_map["/camera_spinnaker_node/stop"]);
            }
            std::lock_guard<std::mutex> lock(_mutex);
            modbus_reply(ctx, query, rc, mb_mapping);
        }
    }
    
private:
    ModbusRos* _node;
    modbus_t* ctx = NULL;
    int header_length;
    modbus_mapping_t* mb_mapping = NULL;
    int sock = -1;
    unsigned char query[MODBUS_TCP_MAX_ADU_LENGTH];
    std::mutex _mutex;
};

ModbusRos::ModbusRos(const rclcpp::NodeOptions& options) : Node("modbus_ros_node", options)
{
    _init = std::thread(&ModbusRos::_Init, this);
}

ModbusRos::~ModbusRos()
{
    _init.join();

    _sub.reset();
    _impl.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "modbus_ros destroyed successfully");
}

void ModbusRos::_Init() try
{
    _InitializeParameters();

    _UpdateParameters();
    
    while(rclcpp::ok())
    {
        auto srvs = this->get_service_names_and_types();
        auto pos = std::find_if(srvs.begin(), srvs.end(), [](const std::pair<std::string, std::vector<std::string>>& p)
        {
            return EndsWith(p.first, "/camera_spinnaker_node/start");
        });

        if(pos == srvs.end())
        {
            std::this_thread::sleep_for(200ms);
            continue;
        }
        else
        {
            _map[pos->first] = nullptr;
            break;
        }
    }
    
    while(rclcpp::ok())
    {
        auto srvs = this->get_service_names_and_types();
        auto pos = std::find_if(srvs.begin(), srvs.end(), [](const std::pair<std::string, std::vector<std::string>>& p)
        {
            return EndsWith(p.first, "/camera_spinnaker_node/stop");
        });

        if(pos == srvs.end())
        {
            std::this_thread::sleep_for(200ms);
            continue;
        }
        else
        {
            _map[pos->first] = nullptr;
            break;
        }
    }

    for(auto& p : _map)
    {
        const auto& n = p.first;
        p.second = this->create_client<std_srvs::srv::Trigger>(n);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Services ready [%s]", n.c_str());
    }

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<shared_interfaces::msg::ModbusCoord>(_subName, 10, std::bind(&ModbusRos::_Sub, this, std::placeholders::_1));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "modbus_ros initialized successfully");
}
catch(const std::exception& e)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros initializer: %s", e.what());
    rclcpp::shutdown();
}
catch(...)
{
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros initializer: unknown");
    rclcpp::shutdown();
}

void ModbusRos::_Sub(shared_interfaces::msg::ModbusCoord::UniquePtr ptr) try
{
    _impl->Update(ptr->a, ptr->b, ptr->c, ptr->d);
}
catch(const std::exception& e)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros subscription: %s", e.what());
}
catch(...)
{
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in modbus_ros subscription: unknown");
}

void ModbusRos::_InitializeParameters()
{
    //this->declare_parameter("");
}

void ModbusRos::_UpdateParameters()
{
    //this->get_parameter("", );
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(modbus_ros::ModbusRos)

