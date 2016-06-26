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

#include <hector_dynamixel_ros_control_wrapper/hector_dynamixel_ros_control_wrapper.hpp>

#include <controller_manager/controller_manager.h>

namespace hector_dynamixel_ros_control_wrapper
{

HectorDynamixelRosControlWrapper::HectorDynamixelRosControlWrapper(ros::NodeHandle &nh)
{
    ROS_INFO("HectorDynamixelRosControlWrapper()");
    ros::NodeHandle joints_nh = ros::NodeHandle(nh, "joints");
    XmlRpc::XmlRpcValue joints_params;
    joints_nh.getParam("", joints_params);
    ROS_DEBUG("Joints params: %s.", joints_params.toXml().c_str());

    XmlRpc::XmlRpcValue::iterator xml_it;
    for (xml_it = joints_params.begin(); xml_it != joints_params.end(); ++xml_it)
    {
        XmlRpc::XmlRpcValue joint_id = xml_it->first;
        ROS_DEBUG("Found joint: %s", joint_id.toXml().c_str());
        XmlRpc::XmlRpcValue joint_values = xml_it->second;

        std::string joint_name = joint_id;
        std::string topic_name = joint_id;
        double offset = 0;
        std::string joint_type = "normal";

        if (joint_values.hasMember("topic_name"))
        {
            topic_name = static_cast<std::string>(joint_values["topic_name"]);
            ROS_DEBUG("Got joint name: %s", joint_name.c_str());
        }
        
        if (joint_values.hasMember("joint_name"))
        {
            joint_name = static_cast<std::string>(joint_values["joint_name"]);
            ROS_DEBUG("Got joint name: %s", joint_name.c_str());
        }


        if (joint_values.hasMember("offset"))
        {
            offset = joint_values["offset"];
            ROS_DEBUG("Got joint offset: %f", offset);
        }

        if (joint_values.hasMember("type"))
        {
            joint_type = static_cast<std::string>(joint_values["type"]);
            ROS_DEBUG("Got joint type: %s", joint_type.c_str());
        }

        if(joint_type == "normal"){
            joint_name_vector_.push_back(joint_name);
        }else if(joint_type == "fake"){
            fake_joint_name_vector_.push_back(joint_name);
        }else{
            ROS_ERROR("invalid joint type %s for joint %s", joint_type.c_str(), joint_id.toXml().c_str());
            continue;
        }

        joint_offset[joint_name] = offset;
        topic_name_map_[joint_name] = topic_name;

    }

    for(unsigned int i=0; i<joint_name_vector_.size(); i++)
    {
        setupJoint(joint_name_vector_[i], true);
    }

    for(unsigned int i=0; i<fake_joint_name_vector_.size(); i++)
    {
        setupJoint(fake_joint_name_vector_[i], false);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    subscriber_spinner_.reset(new ros::AsyncSpinner(1, &subscriber_queue_));
    subscriber_spinner_->start();

    ROS_INFO("HectorDynamixelRosControlWrapper() --done");

}

void HectorDynamixelRosControlWrapper::setupJoint(std::string joint_name, bool withTopics){
    joint_positions_[joint_name] = 0.0;
    joint_velocitys_[joint_name] = 0.0;
    joint_efforts_[joint_name] = 0.0;

    if(withTopics){
        std::string topic_name = topic_name_map_[joint_name];
        joint_cmd_pubs_[joint_name] = nh_.advertise<std_msgs::Float64>("/" + topic_name + "/command", 5);

        ros::Subscriber sub = nh_.subscribe("/" + topic_name + "/state", 1, &HectorDynamixelRosControlWrapper::jointStateCallback, this);
        joint_state_subs_[joint_name] = sub;

        nh_.setCallbackQueue(&subscriber_queue_);
    }

    hardware_interface::JointStateHandle state_handle(joint_name, &joint_positions_[joint_name], &joint_velocitys_[joint_name], &joint_efforts_[joint_name]);
    joint_state_interface_.registerHandle(state_handle);

    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(joint_name), &joint_pos_cmds_[joint_name]);
    position_joint_interface_.registerHandle(pos_handle);
}

void HectorDynamixelRosControlWrapper::cleanup()
{
    subscriber_spinner_->stop();
}

void HectorDynamixelRosControlWrapper::read(ros::Time time, ros::Duration period)
{
    if(received_joint_states_.size()<joint_name_vector_.size()){
	ROS_ERROR_THROTTLE(1,"Trying to read joints but received joint states number is too less");
	ROS_ERROR_THROTTLE(1, "Expected size %d but was %d", joint_name_vector_.size(), received_joint_states_.size());
        return;
    }
    for(unsigned int i=0; i<joint_name_vector_.size(); i++)
    {
	//ROS_INFO_THROTTLE(1,"processing %s", joint_name_vector_[i].c_str());
        joint_positions_[joint_name_vector_[i]] = received_joint_states_[joint_name_vector_[i]]->current_pos - joint_offset[joint_name_vector_[i]];
    }

    for(unsigned int i=0; i<fake_joint_name_vector_.size(); i++)
    {
        //take last value
        joint_positions_[fake_joint_name_vector_[i]] = _fake_joint_values[fake_joint_name_vector_[i]];
    }
}

void HectorDynamixelRosControlWrapper::write(ros::Time time, ros::Duration period)
{

    ROS_DEBUG("write()");
    for(unsigned int i=0; i<joint_name_vector_.size(); i++)
    {
        std_msgs::Float64 msg;
        msg.data = joint_pos_cmds_[joint_name_vector_[i]] + joint_offset[joint_name_vector_[i]];
        joint_cmd_pubs_[joint_name_vector_[i]].publish(msg);
    }
    
    ROS_DEBUG("write() -- fake_joints");
    for(unsigned int i=0; i<fake_joint_name_vector_.size(); i++)
    {
        //remember last joint value for fake 6. dof
        _fake_joint_values[fake_joint_name_vector_[i]] = joint_pos_cmds_[joint_name_vector_[i]] + joint_offset[joint_name_vector_[i]];
    }
    ROS_DEBUG("write() --done");
}

void HectorDynamixelRosControlWrapper::jointStateCallback(const dynamixel_msgs::JointStateConstPtr& dyn_joint_state)
{

    received_joint_states_[dyn_joint_state->name] = dyn_joint_state;
}

}

int main(int argc, char** argv){

    try{
        ROS_INFO("starting");
        ros::init(argc, argv, "hector_dynamixel_ros_control_wrapper");

        ros::NodeHandle pnh("~");

        hector_dynamixel_ros_control_wrapper::HectorDynamixelRosControlWrapper hector_dynamixel_ros_control_wrapper(pnh);

        ROS_DEBUG("ControllerManager - setup");
        controller_manager::ControllerManager cm(&hector_dynamixel_ros_control_wrapper, pnh);
        ROS_DEBUG("ControllerManager - done");

        ros::AsyncSpinner spinner(4);
        spinner.start();

        ros::Rate loop_rate(50);

        ros::Time last_time = ros::Time::now();

        ROS_DEBUG("pre loop");
        while (ros::ok())
        {
            //ROS_INFO("in main loop");
            loop_rate.sleep();

            ros::Time current_time = ros::Time::now();
            ros::Duration elapsed_time = current_time - last_time;
            last_time = current_time;

            //ROS_INFO("before read");
            hector_dynamixel_ros_control_wrapper.read(current_time, elapsed_time);
            //ROS_INFO("after read");

            //ROS_INFO("before cm.update");
            cm.update(current_time, elapsed_time);
            //ROS_INFO("after cm.update");

            //ROS_INFO("before write");
            hector_dynamixel_ros_control_wrapper.write(current_time, elapsed_time);
            //ROS_INFO("after write");
        }

        hector_dynamixel_ros_control_wrapper.cleanup();
    }
    catch(...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
