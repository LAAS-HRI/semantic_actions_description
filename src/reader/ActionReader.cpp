#include "reader/ActionReader.h"

#include "yaml-cpp/yaml.h"

#include <iostream>

#ifndef COLOR_OFF
#define COLOR_OFF     "\x1B[0m"
#endif
#ifndef COLOR_RED
#define COLOR_RED     "\x1B[0;91m"
#endif
#ifndef COLOR_GREEN
#define COLOR_GREEN   "\x1B[1;92m"
#endif

bool ActionReader::load(bool log)
{
    clear();

    std::string yaml_str;
    if(nh_->getParam(ros_param_, yaml_str))
    {
        YAML::Node actions = YAML::Load(yaml_str);
        double analyse_step = 100 / actions.size();
        double analyse_progress = 0;

        for(YAML::const_iterator it = actions.begin(); it != actions.end(); ++it)
        {
            std::vector<std::string> errors;
            action_t* action = nullptr;
            if(actions_.find(it->first.as<std::string>()) == actions_.end())
                action = new action_t(it->first.as<std::string>());
            else
            {
                action = actions_[it->first.as<std::string>()];
                if(action->defined)
                    errors.emplace_back(action->name + " : is defined twice");
            }
            action->defined = true;

            if(log)
                std::cout << "[" << std::setw(3) << std::setprecision(0) << std::fixed << analyse_progress << "%] Analyse action " << action->name << std::endl;

            auto action_description = it->second;
            if(action_description.Type() != YAML::NodeType::Map)
                errors.emplace_back(action->name + " : the description is not in form of a map");
            else
            {
                for(YAML::const_iterator descr_it = action_description.begin(); descr_it != action_description.end(); ++descr_it)
                {
                    if(descr_it->first.as<std::string>() == "type")
                    {
                        if(descr_it->first.Type() == YAML::NodeType::Scalar)
                        {
                            if(actions_.find(descr_it->second.as<std::string>()) == actions_.end())
                                actions_[descr_it->second.as<std::string>()] = new action_t(descr_it->second.as<std::string>());

                            action->type = actions_[descr_it->second.as<std::string>()];
                            actions_[descr_it->second.as<std::string>()]->children.push_back(action);
                        }
                        else
                            errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not in form of scalar");
                    }
                    else if(descr_it->first.as<std::string>() == "agent")
                    {
                        if(descr_it->first.Type() == YAML::NodeType::Scalar)
                            action->agent_type = descr_it->second.as<std::string>();
                        else
                            errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not in form of scalar");
                    }
                    else if(descr_it->first.as<std::string>() == "description")
                    {
                        if(descr_it->first.Type() == YAML::NodeType::Scalar)
                            action->description = descr_it->second.as<std::string>();
                        else
                            errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not in form of scalar");
                    }
                    else if(descr_it->first.as<std::string>() == "verbalizations")
                    {
                        if(descr_it->first.Type() == YAML::NodeType::Scalar)
                            action->verbalizations.emplace_back(descr_it->second.as<std::string>());
                        else if(descr_it->first.Type() == YAML::NodeType::Sequence)
                        {
                            for(auto verbalization : descr_it->second)
                                action->verbalizations.emplace_back(verbalization.as<std::string>());
                        }
                        else
                            errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not in form of scalar or sequence");
                    }
                    else if(descr_it->first.as<std::string>() == "parameters")
                    {
                        if(descr_it->second.Type() == YAML::NodeType::Map)
                        {
                            auto parameters = descr_it->second;
                            for(auto param_it = parameters.begin(); param_it != parameters.end(); ++param_it)
                            {
                                if(param_it->second.Type() == YAML::NodeType::Scalar)
                                    action->parameters[param_it->first.as<std::string>()] = param_it->second.as<std::string>();
                                else
                                    errors.emplace_back(action->name + " : parameter " + param_it->first.as<std::string>() + " is not in form of scalar");
                            }
                        }
                        else
                            errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not in form of map");
                    }
                    else
                      errors.emplace_back(action->name + " : " + descr_it->first.as<std::string>() + " is not a valid keyword");
                }
            }

            actions_[action->name] = action;
            analyse_progress += analyse_step;

            if(log)
                for(auto error : errors)
                  displayError(error);
        }

        for(auto action : actions_)
        {
            if(action.second->type == nullptr)
              actions_roots_.push_back(action.second);
        }

        if(log)
        {
            for(auto action : actions_)
            {
                if(action.second->defined == false)
                  displayError(action.first + " is use but not defined");
            }
            std::cout << COLOR_GREEN << "[100%] Actions analysed" << COLOR_OFF << std::endl;
        }

        return true;
    }
    else
        return false;
}

void ActionReader::displayError(const std::string& error)
{
  std::cout << "[ERROR] " << COLOR_RED << error << COLOR_OFF << std::endl;
  nb_errors_++;
}
