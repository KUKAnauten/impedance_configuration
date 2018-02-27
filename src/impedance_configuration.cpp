/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Niklas Schaefer */

#include <iimoveit/robot_interface.h>
#include <iiwa_ros.h>
#include <iiwa_ros/conversions.h>
#include <iiwa_msgs/CartesianQuantity.h>
#include <iiwa_msgs/JointQuantity.h>

namespace impedance_configuration {


class ImpedanceConfiguration : public iimoveit::RobotInterface {
public:
  ImpedanceConfiguration(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame)
      : RobotInterface(node_handle, planning_group, base_frame) {	

		iiwa_initial_joint_positions_.joint_names.resize(7);
		iiwa_initial_joint_positions_.joint_names = RobotInterface::getJointNames();
		iiwa_initial_joint_positions_.points.resize(1);
		iiwa_initial_joint_positions_.points[0].positions.resize(7);
		iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -44.2626;
		iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (16.7988 - 90.0);
		iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -1.0 * -38.9998;
		iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -67.7357;
		iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (-1.0 * 61.3977 + 90.0); 
		iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * -11.0314; 
		iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;

		iiwa_ros_object_.init();
		//smart_servo_service_caller = iiwa_ros_object_.getSmartServoService();
  }

	void moveToInitialJointPositions() {
		planAndMove(iiwa_initial_joint_positions_.points[0].positions, std::string("initial joint positions"));
	}

	bool setJointImpedanceMode(const iiwa_msgs::JointQuantity& joint_stiffness, const iiwa_msgs::JointQuantity& joint_damping) {
		//return smart_servo_service_caller.setJointImpedanceMode(joint_stiffness, joint_damping);
		return iiwa_ros_object_.getSmartServoService().setJointImpedanceMode(joint_stiffness, joint_damping);
	}

	bool setPositionControlMode() {
		return iiwa_ros_object_.getSmartServoService().setPositionControlMode();
	} 	

private:
	iiwa_ros::iiwaRos iiwa_ros_object_;
	//SmartServoService smart_servo_service_caller;
	trajectory_msgs::JointTrajectory iiwa_initial_joint_positions_;
	
};
} // namespace impedance_configuration

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_impedance_configuration");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

	iiwa_msgs::JointQuantity joint_stiffness1 = iiwa_ros::jointQuantityFromDouble(250, 250, 250, 250, 250, 250, 250);
	iiwa_msgs::JointQuantity joint_damping1 = iiwa_ros::jointQuantityFromDouble(0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8);
	iiwa_msgs::JointQuantity joint_stiffness2 = iiwa_ros::jointQuantityFromDouble(150, 150, 150, 150, 150, 150, 150);
//	iiwa_msgs::JointQuantity joint_damping2;
	

	impedance_configuration::ImpedanceConfiguration impedance_setter(&node_handle, "manipulator", "world");

	//impedance_setter.moveToInitialJointPositions();
	impedance_setter.waitForApproval();
	ROS_INFO_NAMED("impedance_configuration", "hi");
	impedance_setter.setJointImpedanceMode(joint_stiffness1, joint_damping1);
	impedance_setter.waitForApproval();
	impedance_setter.setJointImpedanceMode(joint_stiffness2, joint_damping1);
	impedance_setter.waitForApproval();
	impedance_setter.setJointImpedanceMode(joint_stiffness1, joint_damping1);
	impedance_setter.waitForApproval();
	impedance_setter.setPositionControlMode();
	ROS_INFO_NAMED("impedance_configuration", "bye");

  ros::Rate rate(10);
  while(ros::ok()) {
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
