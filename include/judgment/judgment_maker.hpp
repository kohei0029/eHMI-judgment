#include <ros/ros.h>

#include <std_msgs/String.h>

#include <interactive_rmpc/RobotState.h>
#include <interactive_rmpc/PedestrianState.h>
#include <interactive_rmpc/MPCStates.h>

class JudgmentMaker
{
public:
    JudgmentMaker();
    ~JudgmentMaker();
private:
    ros::NodeHandle _nh;
    ros::NodeHandle _pnh;
    ros::Subscriber _mpc_result_sub;
    ros::Publisher _judgment_pub;
    
    void mpc_result_callback(const interactive_rmpc::MPCStates::ConstPtr& msg);
    void add_interaction_judgment(const interactive_rmpc::MPCStates& mpc_states, std_msgs::String& judgment);
};