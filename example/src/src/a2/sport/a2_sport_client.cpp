#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "a2/a2_sport_client.hpp"

struct TestOption {
  std::string name;
  int id;
};

static const std::vector<TestOption> option_list = {
    {"damp", 0},
    {"balance_stand", 1},
    {"stop_move", 2},
    {"stand_down", 3},
    {"recovery_stand", 4},
    {"move", 5},
    {"switch_gait", 6},
    {"speed_level", 7},
    {"get_state", 8},
    {"recovery_switch", 9},
    {"body_height", 10},
    {"stand_up", 11},
    {"enter_leftside_gait", 12},
    {"exit_leftside_gait", 13},
    {"enter_handstand", 14},
    {"exit_handstand", 15},
    {"front_flip", 16},
    {"back_flip", 17},
    {"pose", 18},
    {"euler", 19},
};

static int ConvertToInt(const std::string& str) {
  try {
    return std::stoi(str);
  } catch (const std::exception&) {
    return -1;
  }
}

class UserInterface {
 public:
  void terminalHandle() {
    std::string input;
    std::getline(std::cin, input);

    if (input == "list") {
      for (const auto& option : option_list) {
        std::cout << option.name << ", id: " << option.id << std::endl;
      }
      return;
    }

    for (const auto& option : option_list) {
      if (input == option.name || ConvertToInt(input) == option.id) {
        test_option_->id = option.id;
        test_option_->name = option.name;
        std::cout << "Test: " << test_option_->name
                  << ", test_id: " << test_option_->id << std::endl;
      }
    }
  }

  TestOption* test_option_{nullptr};
};

class A2SportClientNode : public rclcpp::Node {
 public:
  A2SportClientNode()
      : Node("a2_sport_client_node"), sport_client_(this) {
    test_option_.id = 1;
    user_interface_.test_option_ = &test_option_;
    t1_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      run();
    });
  }

  void run() {
    std::cout << "Input \"list\" to list all test option ..." << std::endl;
    while (rclcpp::ok()) {
      auto time_start = std::chrono::high_resolution_clock::now();
      static constexpr auto dt = std::chrono::microseconds(20000);  // 50Hz

      user_interface_.terminalHandle();

      unitree_api::msg::Request req;
      switch (test_option_.id) {
        case 0:
          sport_client_.Damp(req);
          break;
        case 1:
          sport_client_.BalanceStand(req);
          break;
        case 2:
          sport_client_.StopMove(req);
          break;
        case 3:
          sport_client_.StandDown(req);
          break;
        case 4:
          sport_client_.RecoveryStand(req);
          break;
        case 5:
          sport_client_.Move(req, 0.0F, 0.0F, 0.5F);
          break;
        case 6:
          sport_client_.SwitchGait(req, 0);
          break;
        case 7:
          sport_client_.SpeedLevel(req, 1);
          break;
        case 8: {
          std::map<std::string, std::string> state_map;
          (void)sport_client_.GetState(state_map);
          std::cout << "fsm_id: " << state_map["fsm_id"] << std::endl;
          std::cout << "fsm_name: " << state_map["fsm_name"] << std::endl;
          std::cout << "speed_level: " << state_map["speed_level"] << std::endl;
          std::cout << "auto_recovery_switch: "
                    << state_map["auto_recovery_switch"] << std::endl;
          std::cout << "process_state: " << state_map["process_state"]
                    << std::endl;
          break;
        }
        case 9:
          sport_client_.SetAutoRecovery(req, 0);
          break;
        case 10:
          sport_client_.BodyHeight(req, 0.3F);
          break;
        case 11:
          sport_client_.StandUp(req);
          break;
        case 12:
          sport_client_.LeftSideGait(req, 1);
          break;
        case 13:
          sport_client_.LeftSideGait(req, 0);
          break;
        case 14:
          sport_client_.HandStand(req, 1);
          break;
        case 15:
          sport_client_.HandStand(req, 0);
          break;
        case 16:
          sport_client_.FrontFlip(req);
          break;
        case 17:
          sport_client_.BackFlip(req);
          break;
        case 18:
          sport_client_.BodyPosition(req, 0.2F, 0.2F, -0.2F, 0.2F);
          break;
        case 19:
          sport_client_.Euler(req, 0.2F, 0.3F, 0.3F);
          break;
        default:
          break;
      }

      std::this_thread::sleep_until(time_start + dt);
    }
  }

 private:
  unitree::robot::a2::SportClient sport_client_;
  TestOption test_option_;
  UserInterface user_interface_;
  std::thread t1_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<A2SportClientNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

