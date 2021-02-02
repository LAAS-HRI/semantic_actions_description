#ifndef ACTIONREADER_H
#define ACTIONREADER_HREADER_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>

struct action_t
{
    action_t(const std::string& name) : name(name)
    {
        type = nullptr;
        defined = false;
    }

    std::string name;
    action_t* type;
    std::vector<action_t*> children;
    std::string description;
    std::vector<std::string> verbalizations;
    std::map<std::string, std::string> parameters;
    std::string agent_type;
    bool defined;

    void display()
    {
        if(description != "")
            std::cout << std::endl << "// " << description << std::endl;
        std::cout << name << " : " << std::endl;
        if(type != nullptr)
            std::cout << "\ttype : " << type->name << std::endl;
        if(agent_type != "")
            std::cout << "\tagent : " << agent_type << std::endl;
        if(parameters.size())
            std::cout << "\tparameters : " << std::endl;
        for(auto param : parameters)
            std::cout << "\t\t" << param.first << " : " << param.second << std::endl;
        if(verbalizations.size())
            std::cout << "\tverbalizations" << std::endl;
        for(auto verbalization : verbalizations)
            std::cout << "\t\t" << verbalization << std::endl;
    }
};

class ActionReader
{
public:
    ActionReader(ros::NodeHandle* nh, const std::string& ros_param)
    {
        nh_ = nh;
        ros_param_ = ros_param;
    }

    ~ActionReader()
    {
        clear();
    }

    bool load(bool log = false);

    void clear()
    {
        for(auto action : actions_)
            delete action.second;
        actions_.clear();
        actions_roots_.clear();
        nb_errors_ = 0;
    }

    std::map<std::string, action_t*> getActions() { return actions_; }
    std::vector<action_t*> getRootActions() { return actions_roots_; }
    size_t getNbError() { return nb_errors_; }

private:
    ros::NodeHandle* nh_;
    std::string ros_param_;
    std::map<std::string, action_t*> actions_;
    std::vector<action_t*> actions_roots_;
    size_t nb_errors_;

    void displayError(const std::string& error);
};

#endif // ACTIONREADER_H
