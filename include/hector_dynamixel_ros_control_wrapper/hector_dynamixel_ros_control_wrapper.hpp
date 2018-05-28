/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Christian Rose
 *                      Team Hector,
 *                      Technische Universität Darmstadt
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
 *   * Neither the name of Technische Universität Darmstadt nor the names of
 *     its contributors may be used to endorse or promote products derived
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
 *
 *********************************************************************/

#ifndef hector_dynamixel_ros_control_wrapper_hpp___
#define hector_dynamixel_ros_control_wrapper_hpp___

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <hardware_interface/robot_hw.h>

#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

#include <ros/callback_queue.h>

namespace hector_dynamixel_ros_control_wrapper
{

class HectorDynamixelRosControlWrapper : public hardware_interface::RobotHW
{
public:
    HectorDynamixelRosControlWrapper(ros::NodeHandle &nh);

    void cleanup();

    void read(ros::Time time, ros::Duration period);
    void write(ros::Time time, ros::Duration period);


private:
    void setupJoint(std::string joint_name, bool withTopics);
    void jointStateCallback(const dynamixel_msgs::JointStateConstPtr& dyn_joint_state);

    ros::NodeHandle nh_;

    std::vector<std::string> joint_name_vector_;
    std::map<std::string, std::string> topic_name_map_;
    std::vector<std::string> fake_joint_name_vector_;

    std::map<std::string, double> joint_positions_;
    std::map<std::string, double> joint_pos_cmds_;
    std::map<std::string, double> joint_velocitys_;
    std::map<std::string, double> joint_efforts_;

    std::map<std::string, ros::Publisher> joint_cmd_pubs_;
    std::map<std::string, ros::Subscriber> joint_state_subs_;

    std::map<std::string, dynamixel_msgs::JointStateConstPtr> received_joint_states_;

    boost::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
    ros::CallbackQueue subscriber_queue_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;


    std::map<std::string, double> _fake_joint_values;
    std::map<std::string, double> joint_offset;
    
    int invalid_position_counter;
};

}

#endif
