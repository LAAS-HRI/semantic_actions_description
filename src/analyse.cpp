#include "reader/ActionReader.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_actions_reader");

    ros::NodeHandle n;

    std::string actions_param = "/actions_description";

    ActionReader actions_reader(&n, actions_param, false);

    if(actions_reader.load(true) == false)
        std::cout << "The ros parameter " << actions_param << " does not exist" << std::endl;

    std::string tasks_param = "/tasks_description";
    ActionReader tasks_reader(&n, tasks_param, true);

    if(tasks_reader.load(true) == false)
        std::cout << "The ros parameter " << tasks_param << " does not exist" << std::endl;

    return 0;
}
