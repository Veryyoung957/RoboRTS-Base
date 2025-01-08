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
    // if (heartbeat_thread_.joinable())
    // {
    //   heartbeat_thread_.join();
    // }
  }

  void Gimbal::SDK_Init()
  {

    verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>(UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
                                                                                                      MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
    roborts_sdk::cmd_version_id version_cmd;
    version_cmd.version_id = 0;
    Gimbal::shoot_state=roborts_sdk::SHOOT_STOP;
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

    handle_->CreateSubscriber<roborts_sdk::cmd_rpy>(GIMBAL_CMD_SET, CMD_SET_RPY,
                                                        GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                        std::bind(&Gimbal::GimbalTFCallback, this, std::placeholders::_1));

    handle_->CreateSubscriber<visualization_msgs::msg::Marker>(GIMBAL_CMD_SET, CMD_SET_AIMPOSITION,
                                                            GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                            std::bind(&Gimbal::AimPositionCmdCtrlCallback, this, std::placeholders::_1));
    
    handle_->CreateSubscriber<roborts_sdk::cmd_target>(GIMBAL_CMD_SET, CMD_SET_TARGET,
                                                            GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                            std::bind(&Gimbal::AnotherTargetCallback, this, std::placeholders::_1));

    gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                                                                                MANIFOLD1_ADDRESS, GIMBAL_ADDRESS);

    gimbal_cmd_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_cmd>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_CMD,
                                                                                MANIFOLD1_ADDRESS, GIMBAL_ADDRESS);  
  
    target_cmd_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_target>(GIMBAL_CMD_SET, CMD_SET_TARGET,
                                                                                MANIFOLD1_ADDRESS, MANIFOLD2_ADDRESS);

    fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
                                                                                  MANIFOLD1_ADDRESS, GIMBAL_ADDRESS);
    gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                                                                              MANIFOLD1_ADDRESS, GIMBAL_ADDRESS);

    // heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
    //                                                                       MANIFOLD1_ADDRESS, CHASSIS_ADDRESS);
    // heartbeat_thread_ = std::thread([this]
    //                                 {
    //                                 roborts_sdk::cmd_heartbeat heartbeat;
    //                                 heartbeat.heartbeat=0;
    //                                 while(rclcpp::ok()){
    //                                   heartbeat_pub_->Publish(heartbeat);
    //                                   std::this_thread::sleep_for(std::chrono::milliseconds(300));
    //                                 } });

    reset_tracker_client_ = this->create_client<std_srvs::srv::Trigger>("/tracker/reset");
  }

  void Gimbal::ROS_Init()
  {

    // ros subscriber
    ros_sub_cmd_gimbal_angle_ = this->create_subscription<rm_interfaces::msg::GimbalCmd>(
        "armor_solver/cmd_gimbal", rclcpp::SystemDefaultsQoS(),
        std::bind(&Gimbal::GimbalCmdCtrlCallback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
        "another_target", rclcpp::SensorDataQoS(),
        std::bind(&Gimbal::TargetCallback, this, std::placeholders::_1));

    // gimble_angle_pub_= this->create_subscription<rm_interfaces::msg::GimbalCmd>(
    //     "armor_solver/cmd_gimbal", rclcpp::SensorDataQoS(),
    //     std::bind(&Gimbal::TargetCallback, this, std::placeholders::_1));

    aim_position_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);
    latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
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
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_link";
  }

  void Gimbal::GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info)
  {

    rclcpp::Time current_time = this->get_clock()->now();
    tf2::Quaternion q_1;
    q_1.setRPY(0.0, gimbal_info->pitch_ecd_angle / 1800.0 * M_PI, gimbal_info->yaw_ecd_angle / 1800.0 * M_PI);
    //geometry_msgs::msg::Quaternion q = tf2::toMsg(q);
    gimbal_tf_.header.stamp = current_time;
    gimbal_tf_.transform.rotation = tf2::toMsg(q_1);                            
    gimbal_tf_.transform.translation.x = 0;
    gimbal_tf_.transform.translation.y = 0;
    gimbal_tf_.transform.translation.z = 0.15;
    tf_broadcaster_->sendTransform(gimbal_tf_);
    tf2::Quaternion q_2;
    q_2.setRPY(0.0, gimbal_info->pitch_gyro_angle / 1800.0 * M_PI, gimbal_info->yaw_gyro_angle/ 1800.0 * M_PI);
    t.header.stamp = current_time;
    t.transform.rotation = tf2::toMsg(q_2);
    tf_broadcaster_->sendTransform(t);
  }

  void Gimbal::GimbalTFCallback(const std::shared_ptr<roborts_sdk::cmd_rpy> gimbal_tf)
  {
    rclcpp::Time current_time = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_link";
    tf2::Quaternion q;
    q.setRPY(0.0, gimbal_tf->pitch / 1800.0 * M_PI, gimbal_tf->yaw / 1800.0 * M_PI);
    t.header.stamp = current_time;
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
  }

  // void Gimbal::GimbalAngleCtrlCallback(const rm_interfaces::msg::GimbalCmd::ConstPtr &msg)
  // {

  //   roborts_sdk::cmd_gimbal_angle gimbal_angle;
  //   gimbal_angle.ctrl.bit.pitch_mode = 0;
  //   gimbal_angle.ctrl.bit.yaw_mode = 0;
  //   gimbal_angle.pitch = msg->pitch * 1024;
  //   gimbal_angle.yaw = msg->yaw * 1024;

  //   gimbal_angle_pub_->Publish(gimbal_angle);
  //   RCLCPP_INFO(this->get_logger(), "GimbalAngle publish pitch=%f yaw=%f", msg->pitch,msg->yaw);
  // }


  // void Gimbal::GimbalAngleCtrlCallback(const roborts_msgs::msg::GimbalAngle::ConstPtr &msg)
  // {

  //   roborts_sdk::cmd_gimbal_angle gimbal_angle;
  //   gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
  //   gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
  //   gimbal_angle.pitch = msg->pitch_angle * 1800 / M_PI;
  //   gimbal_angle.yaw = msg->yaw_angle * 1800 / M_PI;

  //   gimbal_angle_pub_->Publish(gimbal_angle);
  // }
  

  void Gimbal::GimbalCmdCtrlCallback(const rm_interfaces::msg::GimbalCmd::ConstPtr &msg)
  {

    roborts_sdk::cmd_gimbal_cmd gimbal_cmd;
    roborts_sdk::cmd_shoot_info gimbal_shoot;
    uint16_t default_freq = 1500;

    gimbal_cmd.pitch = msg->pitch * 10;
    gimbal_cmd.yaw = msg->yaw * 10;
    gimbal_cmd.pitch_diff = msg->pitch_diff * 10;
    gimbal_cmd.yaw_diff = msg->yaw_diff * 10;
    gimbal_cmd.distance = msg->distance * 1000;
    gimbal_cmd.fire_advice = msg->fire_advice;

    if(gimbal_cmd.distance<0 && Gimbal::shoot_state!=roborts_sdk::SHOOT_STOP){
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_STOP;
      gimbal_shoot.shoot_add_num = 0;
      gimbal_shoot.shoot_freq = 0;
      gimbal_shoot_pub_->Publish(gimbal_shoot);
      Gimbal::shoot_state=roborts_sdk::SHOOT_STOP;
    }
    else{
      gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_CONTINUOUS;
      gimbal_shoot.shoot_add_num = 1;
      gimbal_shoot.shoot_freq = default_freq;
      gimbal_shoot_pub_->Publish(gimbal_shoot);
      Gimbal::shoot_state=roborts_sdk::SHOOT_CONTINUOUS;
    }

    gimbal_cmd_pub_->Publish(gimbal_cmd);
    
  }

  void Gimbal::RpyCmdCtrlCallback(const std::shared_ptr<roborts_sdk::cmd_rpy> &msg)
  {

    roborts_msgs::msg::Rpy rpy;
    rpy.pitch = msg->pitch * 1800 / M_PI;
    rpy.yaw = msg->yaw * 1800 / M_PI;
    rpy.roll = msg->roll * 1800 / M_PI;

    rpy_pub_->publish(rpy);
  }

  void Gimbal::AimPositionCmdCtrlCallback(const std::shared_ptr<visualization_msgs::msg::Marker> &msg)
  {

    visualization_msgs::msg::Marker aim_position;
    aim_position.pose.position.x = msg->pose.position.x;
    aim_position.pose.position.y = msg->pose.position.y;
    aim_position.pose.position.z = msg->pose.position.z;

    aim_position_pub_->publish(aim_position);
  }

  void Gimbal::TargetCallback(const rm_interfaces::msg::Target::ConstPtr &msg)
  {
    const static std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
      {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

    roborts_sdk::cmd_target target; 
    target.ctrl.bit.tracking = msg->tracking;
    target.ctrl.bit.id = id_unit8_map.at(msg->id);
    target.ctrl.bit.armors_num = msg->armors_num;
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

    RCLCPP_INFO(this->get_logger(), "target_cmd_pub_  send");
    target_cmd_pub_->Publish(target);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  }

  void Gimbal::AnotherTargetCallback(const roborts_sdk::cmd_target::ConstPtr &msg)
  {
    const static std::map<std::string, uint8_t> id_unit8_map{
      {"", 0},  {"outpost", 0}, {"1", 1}, {"1", 1},     {"2", 2},
      {"3", 3}, {"4", 4},       {"5", 5}, {"guard", 6}, {"base", 7}};

    roborts_sdk::cmd_target target; 
    target.ctrl.bit.tracking = msg->tracking;
    target.ctrl.bit.id = id_unit8_map.at(msg->id);
    target.ctrl.bit.armors_num = msg->armors_num;
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

    RCLCPP_INFO(this->get_logger(), "target_cmd_pub_  send");
    target_cmd_pub_->Publish(target);

    std_msgs::msg::Float64 latency;
    latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_DEBUG_STREAM(get_logger(), "Total latency: " + std::to_string(latency.data) + "ms");
    latency_pub_->publish(latency);
  }

  void Gimbal::resetTracker()
  {
    if (!reset_tracker_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "Service not ready, skipping tracker reset");
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reset_tracker_client_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Reset tracker!");
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
