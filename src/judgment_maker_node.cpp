#include "../include/judgment/judgment_maker.hpp"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "judgment_maker");
    JudgmentMaker jm;
    ros::spin();
    return 0;
}