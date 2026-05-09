#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <string>

#include "nlohmann/json.hpp"
#include "patch.hpp"
#include "common/b2_base_client.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"

namespace unitree::robot::a2 {

constexpr int32_t ROBOT_SPORT_API_ID_DAMP = 1001;
constexpr int32_t ROBOT_SPORT_API_ID_BALANCESTAND = 1002;
constexpr int32_t ROBOT_SPORT_API_ID_STOPMOVE = 1003;
constexpr int32_t ROBOT_SPORT_API_ID_STANDUP = 1004;
constexpr int32_t ROBOT_SPORT_API_ID_STANDDOWN = 1005;
constexpr int32_t ROBOT_SPORT_API_ID_RECOVERYSTAND = 1006;
constexpr int32_t ROBOT_SPORT_API_ID_EULER = 1007;
constexpr int32_t ROBOT_SPORT_API_ID_MOVE = 1008;
constexpr int32_t ROBOT_SPORT_API_ID_BODYPOSITION = 1009;
constexpr int32_t ROBOT_SPORT_API_ID_SWITCHGAIT = 1011;
constexpr int32_t ROBOT_SPORT_API_ID_BODYHEIGHT = 1013;
constexpr int32_t ROBOT_SPORT_API_ID_SPEEDLEVEL = 1015;
constexpr int32_t ROBOT_SPORT_API_ID_LEFTSIDEGAIT = 1016;
constexpr int32_t ROBOT_SPORT_API_ID_RIGHTSIDEGAIT = 1017;
constexpr int32_t ROBOT_SPORT_API_ID_HANDSTAND = 1018;
constexpr int32_t ROBOT_SPORT_API_ID_BIPEDSTAND = 1019;
constexpr int32_t ROBOT_SPORT_API_ID_FRONTFLIP = 1020;
constexpr int32_t ROBOT_SPORT_API_ID_BACKFLIP = 1021;
constexpr int32_t ROBOT_SPORT_API_ID_GETSTATE = 1034;
constexpr int32_t ROBOT_SPORT_API_ID_SETAUTORECOVERY = 1040;

class SportClient {
  rclcpp::Node* node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
  BaseClient base_client_;

 public:
  explicit SportClient(rclcpp::Node* node)
      : node_(node),
        base_client_(node_, "/api/sport/request", "/api/sport/response") {
    req_puber_ = node_->create_publisher<unitree_api::msg::Request>(
        "/api/sport/request", rclcpp::QoS(10));
  }

  void Damp(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_DAMP;
    req_puber_->publish(req);
  }

  void BalanceStand(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BALANCESTAND;
    req_puber_->publish(req);
  }

  void StopMove(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STOPMOVE;
    req_puber_->publish(req);
  }

  void StandUp(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDUP;
    req_puber_->publish(req);
  }

  void StandDown(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_STANDDOWN;
    req_puber_->publish(req);
  }

  void RecoveryStand(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RECOVERYSTAND;
    req_puber_->publish(req);
  }

  void Euler(unitree_api::msg::Request& req, float roll, float pitch,
             float yaw) {
    nlohmann::json js;
    js["x"] = roll;
    js["y"] = pitch;
    js["z"] = yaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_EULER;
    req_puber_->publish(req);
  }

  void Move(unitree_api::msg::Request& req, float vx, float vy, float vyaw) {
    nlohmann::json js;
    js["x"] = vx;
    js["y"] = vy;
    js["z"] = vyaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
    req_puber_->publish(req);
  }

  void SwitchGait(unitree_api::msg::Request& req, int gait_type) {
    nlohmann::json js;
    js["data"] = gait_type;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SWITCHGAIT;
    req_puber_->publish(req);
  }

  void SpeedLevel(unitree_api::msg::Request& req, int level) {
    nlohmann::json js;
    js["data"] = level;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SPEEDLEVEL;
    req_puber_->publish(req);
  }

  void BodyHeight(unitree_api::msg::Request& req, float height) {
    nlohmann::json js;
    js["data"] = height;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYHEIGHT;
    req_puber_->publish(req);
  }

  void BodyPosition(unitree_api::msg::Request& req, float x, float y, float z,
                    float yaw) {
    nlohmann::json js;
    js["x"] = x;
    js["y"] = y;
    js["z"] = z;
    js["yaw"] = yaw;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BODYPOSITION;
    req_puber_->publish(req);
  }

  void LeftSideGait(unitree_api::msg::Request& req, int enter) {
    nlohmann::json js;
    js["data"] = enter;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_LEFTSIDEGAIT;
    req_puber_->publish(req);
  }

  void RightSideGait(unitree_api::msg::Request& req, int enter) {
    nlohmann::json js;
    js["data"] = enter;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_RIGHTSIDEGAIT;
    req_puber_->publish(req);
  }

  void HandStand(unitree_api::msg::Request& req, int enter) {
    nlohmann::json js;
    js["data"] = enter;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_HANDSTAND;
    req_puber_->publish(req);
  }

  void BipedStand(unitree_api::msg::Request& req, int enter) {
    nlohmann::json js;
    js["data"] = enter;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BIPEDSTAND;
    req_puber_->publish(req);
  }

  void FrontFlip(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_FRONTFLIP;
    req_puber_->publish(req);
  }

  void BackFlip(unitree_api::msg::Request& req) {
    req.header.identity.api_id = ROBOT_SPORT_API_ID_BACKFLIP;
    req_puber_->publish(req);
  }

  void SetAutoRecovery(unitree_api::msg::Request& req, int switch_on) {
    nlohmann::json js;
    js["data"] = switch_on;
    req.parameter = js.dump();
    req.header.identity.api_id = ROBOT_SPORT_API_ID_SETAUTORECOVERY;
    req_puber_->publish(req);
  }

  int32_t GetState(std::map<std::string, std::string>& state_map) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = ROBOT_SPORT_API_ID_GETSTATE;
    nlohmann::json js;
    auto ret = base_client_.Call(req, js);
    if (ret != 0) {
      return ret;
    }
    for (auto it = js.begin(); it != js.end(); ++it) {
      if (it.value().is_string()) {
        state_map[it.key()] = it.value().get<std::string>();
      } else {
        state_map[it.key()] = it.value().dump();
      }
    }
    return 0;
  }
};

}  // namespace unitree::robot::a2

