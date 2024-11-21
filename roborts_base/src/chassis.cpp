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

#include "chassis.h"

namespace roborts_base{
Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle):
    Module("chassis",handle){
  SDK_Init();
  ROS_Init();
}
Chassis::~Chassis(){
  if(heartbeat_thread_.joinable()){
    heartbeat_thread_.join();
  }
}
void Chassis::SDK_Init(){

  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  verison_client_->AsyncSendRequest(version,
                                    [this](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      RCLCPP_INFO(this->get_logger(),"Chassis Firmware Version: %d.%d.%d.%d",
                                               int(future.get()->version_id>>24&0xFF),
                                               int(future.get()->version_id>>16&0xFF),
                                               int(future.get()->version_id>>8&0xFF),
                                               int(future.get()->version_id&0xFF));
                                    });

  // handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
  //                                                          CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
  //                                                          std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_uwb_info>(COMPATIBLE_CMD_SET, CMD_PUSH_UWB_INFO,
                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                       std::bind(&Chassis::UWBInfoCallback, this, std::placeholders::_1));

  chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
                                                                                MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
                                                                                    MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                        MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
  // heartbeat_thread_ = std::thread([this]{
  //                                   roborts_sdk::cmd_heartbeat heartbeat;
  //                                   heartbeat.heartbeat=0;
  //                                   while(rclcpp::ok()){
  //                                     heartbeat_pub_->Publish(heartbeat);
  //                                     std::this_thread::sleep_for(std::chrono::milliseconds(300));
  //                                   }
  //                                 });
}
void Chassis::ROS_Init(){
  // Publisher
  ros_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  ros_uwb_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("uwb", rclcpp::SystemDefaultsQoS());

  // Subscriber
  ros_sub_cmd_chassis_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&Chassis::ChassisSpeedCtrlCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // ros_sub_cmd_chassis_vel_acc_ = this->create_subscription<geometry_msgs::msg::Twist>(
  //     "cmd_vel_acc", rclcpp::SystemDefaultsQoS(), std::bind(&Chassis::ChassisSpeedAccCtrlCallback, this, std::placeholders::_1));


  //ros_message_init
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";

  odom_tf_.header.frame_id = "odom";
  odom_tf_.child_frame_id = "base_link";

  uwb_data_.header.frame_id = "uwb";
}
void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info){

  rclcpp::Time current_time = this->get_clock()->now();
  odom_.header.stamp = current_time;
  odom_.pose.pose.position.x = chassis_info->position_x_mm/1000.;
  odom_.pose.pose.position.y = chassis_info->position_y_mm/1000.;
  odom_.pose.pose.position.z = 0.0;
  //geometry_msgs::msg::Quaternion q = tf2::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI);
  tf2::Quaternion q;
  q.setRPY(0, 0, chassis_info->gyro_angle / 1800.0 * M_PI);
  odom_.pose.pose.orientation = tf2::toMsg(q);
  odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
  odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
  odom_.twist.twist.angular.z = chassis_info->gyro_rate / 1800.0 * M_PI;
  ros_odom_pub_->publish(odom_);

  odom_tf_.header.stamp = current_time;
  odom_tf_.transform.translation.x = chassis_info->position_x_mm/1000.;
  odom_tf_.transform.translation.y = chassis_info->position_y_mm/1000.;

  odom_tf_.transform.translation.z = 0.0;
  odom_tf_.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(odom_tf_);
}
void Chassis::UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info){

  uwb_data_.header.stamp = this->get_clock()->now();
  uwb_data_.pose.position.x = ((double)uwb_info->x)/100.0;
  uwb_data_.pose.position.y = ((double)uwb_info->y)/100.0;
  uwb_data_.pose.position.z = 0;
  tf2::Quaternion q;
  q.setRPY(0, 0, uwb_info->yaw/ 180.0 * M_PI);
  uwb_data_.pose.orientation = tf2::toMsg(q);
  ros_uwb_pub_->publish(uwb_data_);

}

void Chassis::ChassisSpeedCtrlCallback(const geometry_msgs::msg::Twist::ConstPtr &vel){
  roborts_sdk::cmd_chassis_speed chassis_speed;
  chassis_speed.vx = vel->linear.x*1000;
  chassis_speed.vy = vel->linear.y*1000;
  chassis_speed.vw = vel->angular.z * 1800.0 / M_PI;
  chassis_speed.rotate_x_offset = 0;
  chassis_speed.rotate_y_offset = 0;
  chassis_speed_pub_->Publish(chassis_speed);
}

// void Chassis::ChassisSpeedAccCtrlCallback(const roborts_msgs::msg::TwistAccel::ConstPtr &vel_acc){
//   roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
//   chassis_spd_acc.vx = vel_acc->twist.linear.x*1000;
//   chassis_spd_acc.vy = vel_acc->twist.linear.y*1000;
//   chassis_spd_acc.vw = vel_acc->twist.angular.z * 1800.0 / M_PI;
//   chassis_spd_acc.ax = vel_acc->accel.linear.x*1000;
//   chassis_spd_acc.ay = vel_acc->accel.linear.y*1000;
//   chassis_spd_acc.wz = vel_acc->accel.angular.z * 1800.0 / M_PI;
//   chassis_spd_acc.rotate_x_offset = 0;
//   chassis_spd_acc.rotate_y_offset = 0;
//   chassis_spd_acc_pub_->Publish(chassis_spd_acc);
// }
}
