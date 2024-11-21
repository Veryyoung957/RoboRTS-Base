/****************************************************************************
 *  Copyright (C) 2021 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "gimbal.h"

namespace roborts_base
{
  Gimbal::Gimbal(std::shared_ptr<roborts_sdk::Handle> handle) : Module("gimbal",handle)
  {
    SDK_Init();
    ROS_Init();
  }

  Gimbal::~Gimbal()
  {
    if (heartbeat_thread_.joinable())
    {
      heartbeat_thread_.join();
    }
  }

  void Gimbal::SDK_Init()
  {

    verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>(UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
                                                                                                      MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    roborts_sdk::cmd_version_id version_cmd;
    version_cmd.version_id = 0;
    auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
    verison_client_->AsyncSendRequest(version,
                                      [this](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                             roborts_sdk::cmd_version_id>::SharedFuture future)
                                      {
                                        RCLCPP_INFO(this->get_logger(),"Gimbal Firmware Version: %d.%d.%d.%d",
                                                 int(future.get()->version_id >> 24 & 0xFF),
                                                 int(future.get()->version_id >> 16 & 0xFF),
                                                 int(future.get()->version_id >> 8 & 0xFF),
                                                 int(future.get()->version_id & 0xFF));
                                      });

    handle_->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
                                                            GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                            std::bind(&Gimbal::GimbalInfoCallback, this, std::placeholders::_1));

    handle_->CreateSubscriber<roborts_msgs::msg::Target>(GIMBAL_CMD_SET, CMD_SET_TARGET,
                                                            GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                            std::bind(&Gimbal::TargetCallback, this, std::placeholders::_1));
    gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

    gimbal_cmd_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_cmd>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_CMD,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);                                                                                                                                                    

    latency_cmd_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_latency>(GIMBAL_CMD_SET, CMD_SET_LATENCY,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

    aim_position_cmd_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_aim_position>(GIMBAL_CMD_SET, CMD_SET_AIMPOSITION,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);                                                                                

    fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
                                                                                  MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                                                                              MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

    heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                          MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
                                                                          
    heartbeat_thread_ = std::thread([this]
                                    {
                                    roborts_sdk::cmd_heartbeat heartbeat;
                                    heartbeat.heartbeat=0;
                                    while(rclcpp::ok()){
                                      heartbeat_pub_->Publish(heartbeat);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                    } });
  }

  void Gimbal::ROS_Init()
  {

    // ros subscriber
    ros_sub_cmd_gimbal_angle_ = this->create_subscription<roborts_msgs::msg::GimbalAngle>(
        "cmd_gimbal_angle", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::GimbalAngleCtrlCallback, this, std::placeholders::_1));

    ros_sub_cmd_gimbal_cmd_ = this->create_subscription<roborts_msgs::msg::GimbalCmd>(
        "cmd_gimbal_angle", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::GimbalCmdCtrlCallback, this, std::placeholders::_1));

    ros_sub_latency_cmd_ = this->create_subscription<roborts_msgs::msg::Latency>(
        "cmd_gimbal_angle", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::LatencyCmdCtrlCallback, this, std::placeholders::_1));

    ros_sub_aim_position_cmd_ = this->create_subscription<roborts_msgs::msg::AimingPoint>(
        "cmd_gimbal_angle", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::AimPositionCmdCtrlCallback, this, std::placeholders::_1));

    ros_sub_auto_aim_ = this->create_subscription<roborts_msgs::msg::Target>(
        "cmd_gimbal_angle", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::TargetCallback, this, std::placeholders::_1));
             
    // Services
    // ros_ctrl_fric_wheel_srv_ = this->create_service<roborts_msgs::srv::FricWhl>(
    //     "cmd_fric_wheel", std::bind(&Gimbal::CtrlFricWheelService, this,
    //     std::placeholders::_1, std::placeholders::_2));
    ros_ctrl_fric_wheel_srv_ = this->create_service<roborts_msgs::srv::FricWhl>(
        "cmd_fric_wheel",
        [this](const std::shared_ptr<roborts_msgs::srv::FricWhl::Request> request,
              std::shared_ptr<roborts_msgs::srv::FricWhl::Response> response) {
            this->CtrlFricWheelService(request, response);
    });

    ros_ctrl_shoot_srv_ = this->create_service<roborts_msgs::srv::ShootCmd>(
        "cmd_shoot", 
        [this](const std::shared_ptr<roborts_msgs::srv::ShootCmd::Request> request,
                        std::shared_ptr<roborts_msgs::srv::ShootCmd::Response> response){
                          this->CtrlShootService(request, response);
    });

    // Message Initialization
    gimbal_tf_.header.frame_id = "base_link";
    gimbal_tf_.child_frame_id = "gimbal";
  }

  void Gimbal::GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info)
  {

    rclcpp::Time current_time = this->get_clock()->now();
    tf2::Quaternion q;
    q.setRPY(0.0, gimbal_info->pitch_ecd_angle / 1800.0 * M_PI, gimbal_info->yaw_ecd_angle / 1800.0 * M_PI);
    //geometry_msgs::msg::Quaternion q = tf2::toMsg(q);
    gimbal_tf_.header.stamp = current_time;
    gimbal_tf_.transform.rotation = tf2::toMsg(q);
    gimbal_tf_.transform.translation.x = 0;
    gimbal_tf_.transform.translation.y = 0;
    gimbal_tf_.transform.translation.z = 0.15;
    tf_broadcaster_->sendTransform(gimbal_tf_);
  }

  void Gimbal::GimbalAngleCtrlCallback(const roborts_msgs::msg::GimbalAngle::ConstPtr &msg)
  {

    roborts_sdk::cmd_gimbal_angle gimbal_angle;
    gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
    gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
    gimbal_angle.pitch = msg->pitch_angle * 1800 / M_PI;
    gimbal_angle.yaw = msg->yaw_angle * 1800 / M_PI;

    gimbal_angle_pub_->Publish(gimbal_angle);
  }
  

  void Gimbal::GimbalCmdCtrlCallback(const roborts_msgs::msg::GimbalCmd::ConstPtr &msg)
  {

    roborts_sdk::cmd_gimbal_cmd gimbal_cmd;
    gimbal_cmd.pitch = msg->pitch * 1800 / M_PI;
    gimbal_cmd.yaw = msg->yaw * 1800 / M_PI;
    gimbal_cmd.pitch_diff = msg->pitch_diff;
    gimbal_cmd.yaw_diff = msg->yaw_diff;
    gimbal_cmd.distance = msg->distance;
    gimbal_cmd.fire_advice = msg->fire_advice;

    gimbal_cmd_pub_->Publish(gimbal_cmd);
  }

  void Gimbal::LatencyCmdCtrlCallback(const roborts_msgs::msg::Latency::ConstPtr &msg)
  {

    roborts_sdk::cmd_latency latency;
    latency.pitch = msg->pitch * 1800 / M_PI;
    latency.yaw = msg->yaw * 1800 / M_PI;
    latency.roll = msg->roll * 1800 / M_PI;

    latency_cmd_pub_->Publish(latency);
  }

  void Gimbal::AimPositionCmdCtrlCallback(const roborts_msgs::msg::AimingPoint::ConstPtr &msg)
  {

    roborts_sdk::cmd_aim_position aim_position;
    aim_position.aim_x = msg->aim_x;
    aim_position.aim_y = msg->aim_y;
    aim_position.aim_z = msg->aim_z;

    aim_position_cmd_pub_->Publish(aim_position);
  }

    void Gimbal::TargetCallback(const roborts_msgs::msg::Target::ConstPtr &msg)
  {
    const static std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
      {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

    roborts_sdk::cmd_target target;
    target.tracking = msg->tracking;
    target.id = id_unit8_map.at(msg->id);
    target.armors_num = msg->armors_num;
    // 三维空间中的位置
    target.x = msg->position.x;
    target.y = msg->position.y;
    target.z = msg->position.z;
    target.yaw = msg->yaw;
    // 三维空间中的速度
    target.vx = msg->position.x;
    target.vy = msg->position.y;
    target.vz = msg->position.z;
    target.v_yaw = msg->v_yaw;
    target.r1 = msg->radius_1;
    target.r2 = msg->radius_2;
    target.dz = msg->dz; // 高度差
  }


  bool Gimbal::CtrlFricWheelService(const std::shared_ptr<roborts_msgs::srv::FricWhl::Request> &req,
                                    std::shared_ptr<roborts_msgs::srv::FricWhl::Response> &res)
  {
    roborts_sdk::cmd_fric_wheel_speed fric_speed;
    if (req->open)
    {
      fric_speed.left = 1240;
      fric_speed.right = 1240;
    }
    else
    {
      fric_speed.left = 1000;
      fric_speed.right = 1000;
    }
    fric_wheel_pub_->Publish(fric_speed);
    res->received = true;
    return 0;
  }
  void Gimbal::CtrlShootService(const std::shared_ptr<roborts_msgs::srv::ShootCmd::Request> &req,
                        std::shared_ptr<roborts_msgs::srv::ShootCmd::Response> &res)
  {
    roborts_sdk::cmd_shoot_info gimbal_shoot;
    uint16_t default_freq = 1500;
    switch (static_cast<roborts_sdk::shoot_cmd_e>(req->mode))
    {
    case roborts_sdk::SHOOT_STOP:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_STOP;
      gimbal_shoot.shoot_add_num = 0;
      gimbal_shoot.shoot_freq = 0;
      break;
    case roborts_sdk::SHOOT_ONCE:
      if (req->number != 0)
      {
        gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
        gimbal_shoot.shoot_add_num = req->number;
        gimbal_shoot.shoot_freq = default_freq;
      }
      else
      {
        gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
        gimbal_shoot.shoot_add_num = 1;
        gimbal_shoot.shoot_freq = default_freq;
      }
      break;
    case roborts_sdk::SHOOT_CONTINUOUS:
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_CONTINUOUS;
      gimbal_shoot.shoot_add_num = req->number;
      gimbal_shoot.shoot_freq = default_freq;
      break;
    default:
      return ;
    }
    gimbal_shoot_pub_->Publish(gimbal_shoot);

    res->received = true;
    return ;
  }
}
