#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cctype>
#include <iostream>
#include <map>
#include <poll.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "a2/a2_sport_client.hpp"

namespace {

struct TestOption {
  std::string name;
  int id;
};

const std::vector<TestOption> option_list = {
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

/** Blocks until one line is available or rclcpp shuts down (Ctrl+C). */
bool ReadStdinLine(std::string* line) {
  while (rclcpp::ok()) {
    pollfd pfd{};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    const int pr = poll(&pfd, 1, 200);
    if (pr < 0) {
      if (errno == EINTR) {
        continue;
      }
      return false;
    }
    if (pr == 0) {
      continue;
    }
    if (pfd.revents & (POLLERR | POLLNVAL)) {
      return false;
    }
    if (pfd.revents & POLLIN) {
      return static_cast<bool>(std::getline(std::cin, *line));
    }
  }
  return false;
}

static void Trim(std::string* s) {
  if (!s || s->empty()) {
    return;
  }
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  s->erase(s->begin(), std::find_if(s->begin(), s->end(), not_space));
  s->erase(std::find_if(s->rbegin(), s->rend(), not_space).base(), s->end());
}

/** list / blank / unknown => *run_sport false; matched item => true. */
void ParseMenuLine(const std::string& raw, TestOption* selected,
                   bool* run_sport) {
  *run_sport = false;
  std::string input = raw;
  Trim(&input);
  if (input.empty()) {
    return;
  }
  if (input == "list") {
    for (const auto& it : option_list) {
      std::cout << it.name << ", id: " << it.id << std::endl;
    }
    return;
  }

  int id = -1;
  try {
    id = std::stoi(input);
  } catch (const std::exception&) {
    id = -1;
  }

  for (const auto& it : option_list) {
    if (input == it.name || (id >= 0 && id == it.id)) {
      *selected = it;
      std::cout << "Test: " << selected->name << ", test_id: " << selected->id
                << std::endl;
      *run_sport = true;
      return;
    }
  }

  std::cout << "Unknown command. Input \"list\" or a valid name / id."
            << std::endl;
}

}  // namespace

class A2SportClientNode : public rclcpp::Node {
 public:
  A2SportClientNode()
      : Node("a2_sport_client_node"), sport_client_(this) {
    selection_.id = 1;
    selection_.name = "balance_stand";
    worker_ = std::thread([this] {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      Run();
    });
  }

  ~A2SportClientNode() override {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void Run() {
    std::cout << "Input \"list\" to list all test option ..." << std::endl;
    while (rclcpp::ok()) {
      std::string line;
      if (!ReadStdinLine(&line)) {
        return;
      }
      bool run_sport = false;
      ParseMenuLine(line, &selection_, &run_sport);
      if (!rclcpp::ok()) {
        return;
      }
      if (!run_sport) {
        continue;
      }

      unitree_api::msg::Request req{};
      switch (selection_.id) {
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
          std::cout << "Unknown option." << std::endl;
          break;
      }
    }
  }

  unitree::robot::a2::SportClient sport_client_;
  TestOption selection_;
  std::thread worker_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  {
    auto node = std::make_shared<A2SportClientNode>();
    while (rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  rclcpp::shutdown();
  return 0;
}