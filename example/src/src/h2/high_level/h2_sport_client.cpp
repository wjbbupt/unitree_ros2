#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "h2/h2_sport_client.hpp"

struct ParsedArg
{
    std::string key;
    std::string value;
};

static std::vector<float> StringToFloatVector(const std::string &str)
{
    std::vector<float> result;
    std::stringstream ss(str);
    float num;
    while (ss >> num)
    {
        result.push_back(num);
        ss.ignore();
    }
    return result;
}

static std::map<std::string, std::string> ParseArgs(int argc, char **argv)
{
    // Keep the same "argument-driven" behavior as:
    std::map<std::string, std::string> args = {{"network_interface", "lo"}};

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg.rfind("--", 0) != 0)
        {
            continue;
        }

        const size_t pos = arg.find('=');
        std::string key;
        std::string value;
        if (pos != std::string::npos)
        {
            key = arg.substr(2, pos - 2);
            value = arg.substr(pos + 1);
            if (!value.empty() && value.front() == '"' && value.back() == '"')
            {
                value = value.substr(1, value.length() - 2);
            }
        }
        else
        {
            key = arg.substr(2);
            value = "";
        }

        if (args.find(key) != args.end())
        {
            args[key] = value;
        }
        else
        {
            args.insert({key, value});
        }
    }

    return args;
}

static int32_t ExecuteCommand(unitree::robot::h2::H2LocoClient &client, const std::string &key,
                             const std::string &value)
{
    int32_t ret = 0;

    if (key == "get_fsm_id")
    {
        int fsm_id = -1;
        ret = client.GetFsmId(fsm_id);
        if (ret == 0)
            std::cout << "current fsm_id: " << fsm_id << std::endl;
    }
    else if (key == "get_fsm_mode")
    {
        int fsm_mode = -1;
        ret = client.GetFsmMode(fsm_mode);
        if (ret == 0)
            std::cout << "current fsm_mode: " << fsm_mode << std::endl;
    }
    else if (key == "set_fsm_id")
    {
        const int fsm_id = std::stoi(value);
        ret = client.SetFsmId(fsm_id);
        if (ret == 0)
            std::cout << "set fsm_id to " << fsm_id << std::endl;
    }
    else if (key == "set_velocity")
    {
        std::vector<float> param = StringToFloatVector(value);
        const size_t param_size = param.size();
        float vx = 0.F, vy = 0.F, omega = 0.F, duration = 1.F;
        if (param_size == 3)
        {
            vx = param.at(0);
            vy = param.at(1);
            omega = param.at(2);
            duration = 1.F;
        }
        else if (param_size == 4)
        {
            vx = param.at(0);
            vy = param.at(1);
            omega = param.at(2);
            duration = param.at(3);
        }
        else
        {
            std::cerr << "Invalid param size for method SetVelocity: " << param_size << std::endl;
            return -1;
        }
        ret = client.SetVelocity(vx, vy, omega, duration);
        if (ret == 0)
            std::cout << "set velocity to " << value << std::endl;
    }
    else if (key == "damp")
    {
        ret = client.Damp();
    }
    else if (key == "start")
    {
        ret = client.Start();
    }
    else if (key == "stand_up")
    {
        ret = client.StandUp();
    }
    else if (key == "zero_torque")
    {
        ret = client.ZeroTorque();
    }
    else if (key == "stop_move")
    {
        ret = client.StopMove();
    }
    else if (key == "switch_move_mode")
    {
        bool flag = false;
        if (value == "true")
        {
            flag = true;
        }
        else if (value == "false")
        {
            flag = false;
        }
        else
        {
            std::cerr << "invalid argument: " << value << std::endl;
            return -1;
        }
        ret = client.SwitchMoveMode(flag);
    }
    else if (key == "move")
    {
        std::vector<float> param = StringToFloatVector(value);
        const size_t param_size = param.size();
        if (param_size != 3)
        {
            std::cerr << "Invalid param size for method Move: " << param_size << std::endl;
            return -1;
        }
        ret = client.Move(param.at(0), param.at(1), param.at(2));
    }
    else if (key == "set_speed_mode")
    {
        const int speed_mode = std::stoi(value);
        ret = client.SetSpeedMode(speed_mode);
        if (ret == 0)
            std::cout << "set speed mode to " << value << std::endl;
    }
    else
    {
        std::cerr << "Unknown command: " << key << std::endl;
        return -1;
    }

    return ret;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("h2_sport_client_node");
    unitree::robot::h2::H2LocoClient client(node.get());

    const auto args = ParseArgs(argc, argv);
    for (const auto &arg_pair : args)
    {
        std::cout << "Processing command: [" << arg_pair.first << "] with param: [" << arg_pair.second
                  << "] ..." << std::endl;
        if (arg_pair.first == "network_interface")
        {
            continue;
        }

        const int32_t ret = ExecuteCommand(client, arg_pair.first, arg_pair.second);
        if (ret != 0)
        {
            std::cerr << "Call failed, ret: " << ret << std::endl;
            rclcpp::shutdown();
            return 1;
        }

        std::cout << "Done!" << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
