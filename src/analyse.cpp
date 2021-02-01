#include "reader/ActionReader.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_actions_reader");

    ros::NodeHandle n;

    std::string param = "/actions_description";

    ActionReader reader(&n, param);

    if(reader.load(true) == false)
        std::cout << "The ros parameter " << param << " does not exist" << std::endl;

    return 0;
}