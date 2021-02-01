#include "reader/ActionReader.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_actions_reader");

    ros::NodeHandle n;

    ActionReader reader(&n, "/actions_description");

    reader.load(true);

    auto actions = reader.getActions();
    for(auto action : actions)
        action.second->display();

    return 0;
}