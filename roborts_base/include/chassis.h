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

#ifndef ROBORTS_BASE_CHASSIS_H
#define ROBORTS_BASE_CHASSIS_H
#include "roborts_sdk.h"
#include "ros_dep.h"
#include "module.h"
#include "utils/factory.h"

namespace roborts_base {
/**
 * @brief ROS API for chassis module
 */
class Chassis: public Module{
 public:
  /**
   * @brief Constructor of chassis including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  explicit Chassis(std::shared_ptr<roborts_sdk::Handle> handle);

  /**
   * @brief Destructor of chassis
   */
  ~Chassis();

 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();

  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**
   * @brief Chassis information callback in sdk
   * @param chassis_info Chassis information
   */
  void ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info);

  /**
   * @brief UWB information callback in sdk
   * @param uwb_info UWB information
   */
  void UWBInfoCallback(const std::shared_ptr<roborts_sdk::cmd_uwb_info> uwb_info);

  /**
   * @brief Chassis speed control callback in ROS
   * @param vel Chassis speed control data
   */
  void ChassisSpeedCtrlCallback(const geometry_msgs::msg::Twist::ConstPtr &vel);

  /**
   * @brief Chassis speed and acceleration control callback in ROS
   * @param vel_acc Chassis speed and acceleration control data
   */
  //void ChassisSpeedAccCtrlCallback(const roborts_msgs::TwistAccel::ConstPtr &vel_acc);

  //! sdk version client
  std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>> verison_client_;

  //! sdk heartbeat thread
  std::thread heartbeat_thread_;
  //! sdk publisher for heartbeat
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;

  //! sdk publisher for chassis speed control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_speed>> chassis_speed_pub_;
  //! sdk publisher for chassis speed and acceleration control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_spd_acc>> chassis_spd_acc_pub_;

  //! ros node handler
  //rclcpp::NodeHandle ros_nh_;
  //! ros subscriber for speed control
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_sub_cmd_chassis_vel_;
  //! ros subscriber for chassis speed and acceleration control
  //rclcpp::Subscriber ros_sub_cmd_chassis_vel_acc_;
  //! ros publisher for odometry information
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ros_odom_pub_;
  //! ros publisher for uwb information
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ros_uwb_pub_;


  //! ros chassis odometry tf
  geometry_msgs::msg::TransformStamped odom_tf_;
  //! ros chassis odometry tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  //! ros odometry message
  nav_msgs::msg::Odometry odom_;
  //! ros uwb message
  geometry_msgs::msg::PoseStamped uwb_data_;
};
// REGISTER_MODULE(Module, "chassis", Chassis, std::shared_ptr<roborts_sdk::Handle>);
}
#endif //ROBORTS_BASE_CHASSIS_H
