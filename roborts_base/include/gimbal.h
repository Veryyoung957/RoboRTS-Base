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

#ifndef ROBORTS_BASE_GIMBAL_H
#define ROBORTS_BASE_GIMBAL_H
#include "roborts_sdk.h"
#include "ros_dep.h"
#include "module.h"
#include "utils/factory.h"

namespace roborts_base {
/**
 * @brief ROS API for gimbal module
 */
class Gimbal: public Module {
 public:
  /**
   * @brief Constructor of gimbal including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  explicit Gimbal(std::shared_ptr<roborts_sdk::Handle> handle);
    /**
   * @brief Destructor of gimbal
   */
  ~Gimbal();
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
   * @brief Gimbal information callback in sdk
   * @param gimbal_info Gimbal information
   */
  void GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info);
  /**
   * @brief Gimbal angle control callback in ROS
   * @param msg Gimbal angle control data
   */
  void GimbalAngleCtrlCallback(const roborts_msgs::msg::GimbalAngle::ConstPtr &msg);
  /**
   * @brief Control friction wheel service callback in ROS
   * @param req Friction wheel control data as request
   * @param res Control result as response
   * @return True if success
   */

  void GimbalTFCallback(const std::shared_ptr<roborts_sdk::cmd_rpy> gimbal_tf);

  void GimbalCmdCtrlCallback(const roborts_msgs::msg::GimbalCmd::ConstPtr &msg);

  void RpyCmdCtrlCallback(const std::shared_ptr<roborts_sdk::cmd_rpy> &msg);
  void AimPositionCmdCtrlCallback(const std::shared_ptr<visualization_msgs::msg::Marker> &msg);

  void TargetCallback(const roborts_msgs::msg::Target::ConstPtr &msg);

  bool CtrlFricWheelService(const std::shared_ptr<roborts_msgs::srv::FricWhl::Request> &req,
                                    std::shared_ptr<roborts_msgs::srv::FricWhl::Response> &res);
  /**
   * @brief Control shoot service callback in ROS
   * @param req Shoot control data as request
   * @param res Control result as response
   * @return True if success
   */
  void CtrlShootService(const std::shared_ptr<roborts_msgs::srv::ShootCmd::Request> &req,
                        std::shared_ptr<roborts_msgs::srv::ShootCmd::Response> &res);

  //
  void resetTracker();

  //! sdk version client
  std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                      roborts_sdk::cmd_version_id>> verison_client_;

  //! sdk heartbeat thread
  std::thread heartbeat_thread_;
  //! sdk publisher for heartbeat
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;


  //! sdk publisher for gimbal angle control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_angle>>     gimbal_angle_pub_;
  //! sdk publisher for frcition wheel control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_fric_wheel_speed>> fric_wheel_pub_;
  //! sdk publisher for gimbal shoot control
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_shoot_info>>       gimbal_shoot_pub_;

  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_cmd>>     gimbal_cmd_pub_;

  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_target>>     target_cmd_pub_;

  // sdk subscription
  // std::shared_ptr<roborts_sdk::Subscription<roborts_sdk::cmd_target>>       auto_aim_cmd_;
  std::shared_ptr<roborts_sdk::Subscription<roborts_sdk::cmd_rpy>>            rpy_cmd_;
  std::shared_ptr<roborts_sdk::Subscription<roborts_sdk::cmd_aim_position>>       aim_position_cmd_;



  //! ros node handler
  //rclcpp::NodeHandle    ros_nh_;
  //! ros subscriber for gimbal angle control
  rclcpp::Subscription<roborts_msgs::msg::GimbalAngle>::SharedPtr    ros_sub_cmd_gimbal_angle_;

  rclcpp::Subscription<roborts_msgs::msg::Target>::SharedPtr    ros_sub_target_;
  //! ros service server for friction wheel control
  rclcpp::Service<roborts_msgs::srv::FricWhl>::SharedPtr ros_ctrl_fric_wheel_srv_;
  //! ros service server for gimbal shoot control
  rclcpp::Service<roborts_msgs::srv::ShootCmd>::SharedPtr ros_ctrl_shoot_srv_;
  //! ros gimbal tf
  geometry_msgs::msg::TransformStamped gimbal_tf_;
  geometry_msgs::msg::TransformStamped t;
  //! ros gimbal tf broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ros gimbalcmd
  rclcpp::Subscription<roborts_msgs::msg::GimbalCmd>::SharedPtr    ros_sub_cmd_gimbal_cmd_ ;

  rclcpp::Publisher<roborts_msgs::msg::Rpy>::SharedPtr   rpy_pub_ ;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr    aim_position_pub_ ;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr    latency_pub_ ;

 
  // ros subscription
  rclcpp::Subscription<roborts_msgs::msg::Rpy>::SharedPtr    ros_sub_rpy_cmd_ ;
  rclcpp::Subscription<roborts_msgs::msg::AimingPoint>::SharedPtr    ros_sub_aim_position_cmd_ ;

  // ros client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr    reset_tracker_client_;

};
// REGISTER_MODULE(Module, "gimbal", Gimbal, std::shared_ptr<roborts_sdk::Handle>);
}
#endif //ROBORTS_BASE_GIMBAL_H
