#include "../include/judgment/judgment_maker.hpp"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <interactive_rmpc/RobotState.h>
#include <interactive_rmpc/PedestrianState.h>
#include <interactive_rmpc/MPCStates.h>

JudgmentMaker::JudgmentMaker(): _nh(""), _pnh("")
{
    std::string mpc_result_in_topic, ehmi_judgment_out_topic;
        _pnh.param<std::string>("mpc_result_in",mpc_result_in_topic,"/mpc/optimal_scenario");
        _pnh.param<std::string>("ehmi_judgment_out", ehmi_judgment_out_topic, "/eHMI/judgment");
    _mpc_result_sub = _nh.subscribe(mpc_result_in_topic, 1, &JudgmentMaker::mpc_result_callback, this);
    _judgment_pub = _nh.advertise<std_msgs::String>(ehmi_judgment_out_topic, 1);
}

JudgmentMaker::~JudgmentMaker(){}

void JudgmentMaker::add_interaction_judgment(const interactive_rmpc::MPCStates& mpc_states, std_msgs::String& judgment)
{
    bool robot_first = false;
    for (const auto& pedestrian : mpc_states.pedestrians)
    {
        if(!pedestrian.is_crossing||pedestrian.robot_ttcps.empty()||pedestrian.pedestrian_ttcps.empty())
        continue;
        if(pedestrian.robot_ttcps[0] <= 0.0 || pedestrian.pedestrian_ttcps[0] <= 0.0 )
        continue;
        for (size_t i = 0; i < pedestrian.robot_ttcps.size(); i++)
        {
            robot_first = pedestrian.robot_ttcps[i] < pedestrian.pedestrian_ttcps[i];
            if (pedestrian.robot_ttcps[i] < 0.0 || pedestrian.pedestrian_ttcps[i] < 0.0)
                break;
        }
    }
    if (robot_first)
    {
        judgment.data = "Robot will go first.";
    }
    else
    {
        judgment.data = "Pedestrian will go first.";
    }
    // ROS_INFO("Judgment: %f", mpc_states.pedestrians.robot_ttcps.size());
}

void JudgmentMaker::mpc_result_callback(const interactive_rmpc::MPCStates::ConstPtr& msg)
{
    std_msgs::String judgement;
    add_interaction_judgment(*msg, judgement);
    ROS_INFO("Judgment: %s", judgement.data.c_str());
    _judgment_pub.publish(judgement);
}